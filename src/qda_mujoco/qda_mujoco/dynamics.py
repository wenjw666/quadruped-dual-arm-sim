"""V2 free-base plant. Joint references produce bounded PD torques, never qpos writes."""
from pathlib import Path
from xml.etree import ElementTree as ET
import hashlib
import json
from copy import deepcopy
import mujoco
import numpy as np
from qda_kinematics.kinematics import LEG_JOINTS, stance_command, AUBO_HOME_LEFT, AUBO_HOME_RIGHT


def build_dynamic_model(source, urdf, srdf, output, training=False):
    source, output = Path(source), Path(output)
    root = ET.parse(source).getroot()
    world = root.find('worldbody')
    robot = next(b for b in world.findall('body') if b.get('name') == 'base_link')
    robot_names = {b.get('name') for b in robot.iter('body')}
    # A single convex hull fills the wrist recess, unlike MoveIt's triangle
    # mesh checks. Preserve its cavity with deterministic convex components.
    import coacd
    import trimesh
    coacd.set_log_level('warn')
    assets=root.find('asset')
    mesh_assets={x.get('name'):x for x in assets.findall('mesh')}
    decomposition={}
    for body in robot.iter('body'):
        if body.get('name') not in ('left_link5','right_link5','left_link7','right_link7'):continue
        for geom in list(body.findall('geom')):
            if geom.get('group')!='1' or geom.get('type')!='mesh':continue
            original=mesh_assets[geom.get('mesh')];mesh_path=Path(original.get('file'))
            digest=hashlib.sha256(mesh_path.read_bytes()+b'coacd-1.0.7-threshold025-seed7').hexdigest()[:20]
            folder=output.parent/'collision_parts'/digest;metadata=folder/'parts.json'
            if not metadata.exists():
                folder.mkdir(parents=True,exist_ok=True)
                mesh=trimesh.load(mesh_path,force='mesh')
                parts=coacd.run_coacd(coacd.Mesh(mesh.vertices,mesh.faces),threshold=.025,max_convex_hull=32,seed=7)
                files=[]
                for i,(vertices,faces) in enumerate(parts):
                    file=folder/f'{i}.obj';trimesh.Trimesh(vertices,faces).export(file);files.append(str(file.resolve()))
                metadata.write_text(json.dumps(files))
            files=json.loads(metadata.read_text());decomposition[body.get('name')]=len(files)
            body.remove(geom)
            for i,file in enumerate(files):
                name=geom.get('name')+f'_part{i}'
                asset=deepcopy(original);asset.set('name',name);asset.set('file',file);assets.append(asset)
                component=deepcopy(geom);component.set('name',name);component.set('mesh',name);body.append(component)
    # Training uses the same articulated robot on an empty plane, with no render loop.
    if training:
        for body in list(world.findall('body')):
            if body is not robot:
                world.remove(body)
    else:
        from .cameras import add_cameras
        add_cameras(robot)
    # Keep the same nominal articulated plant in training and scene deployment.
    # This surrogate is not a physical grasp and is reported separately.
    from qda_kinematics.assembly import GRIPPER_PAD
    wrist=next(b for b in robot.iter('body') if b.get('name')=='left_link7')
    load=ET.SubElement(wrist,'body',name='training_payload',pos=f'0 0 {GRIPPER_PAD}')
    ET.SubElement(load,'inertial',mass='.001',pos='0 0 0',diaginertia='1e-6 1e-6 1e-6')
    for body in robot.iter('body'):
        body.set('gravcomp', '0')
        for geom in body.findall('geom'):
            if geom.get('group') == '1':
                geom.set('contype', '2')
                geom.set('conaffinity', '3')
                geom.set('friction', '.8 .005 .0001')
    contact = ET.SubElement(root, 'contact')
    excluded = set()
    def exclude(a, b):
        pair = tuple(sorted((a, b)))
        if a in robot_names and b in robot_names and a != b and pair not in excluded:
            excluded.add(pair)
            ET.SubElement(contact, 'exclude', body1=a, body2=b)
    for entry in ET.parse(srdf).getroot().findall('disable_collisions'):
        exclude(entry.get('link1'), entry.get('link2'))
    # Same-hand linkage contacts are represented by joint mimic constraints.
    # Other arm/leg/world contacts remain enabled.
    for side in ('left', 'right'):
        fingers = sorted(n for n in robot_names if n.startswith((side+'_robotiq_85_', side+'_finger_')))
        for i, a in enumerate(fingers):
            for b in fingers[i+1:]: exclude(a, b)
    urdf_joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
    actuator = ET.SubElement(root, 'actuator')
    equality = ET.SubElement(root, 'equality')
    motors = []
    for joint in robot.iter('joint'):
        name = joint.get('name')
        original = urdf_joints[name]
        mimic = original.find('mimic')
        if mimic is not None:
            ET.SubElement(equality, 'joint', joint1=name, joint2=mimic.get('joint'),
                          polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                          solref='.01 1')
            continue
        effort = float(original.find('limit').get('effort'))
        if not np.isfinite(effort) or effort <= 0:
            raise ValueError(f'Invalid effort limit: {name}')
        ET.SubElement(actuator, 'motor', name=name+'_motor', joint=name,
                      ctrllimited='true', ctrlrange=f'{-effort} {effort}', gear='1')
        motors.append(name)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(output))
    report = dict(mode='free_base_pd', training_plane=training, robot_mass_kg=float(model.body_subtreemass[model.body('base_link').id]),
                  motor_joints=motors, excluded_body_pairs=sorted(excluded),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),wrist_convex_components=decomposition,
                  limits_source='URDF effort; not calibrated hardware torque-speed envelopes',
                  calibrated_hardware=False)
    output.with_suffix('.dynamic.json').write_text(json.dumps(report, indent=2))
    source_manifest=source.with_suffix('.manifest.json')
    if source_manifest.exists():
        manifest=json.loads(source_manifest.read_text())
        manifest.update(motion='free_base_pd',training_plane=training)
        output.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2))
    return report


class DynamicPlant:
    def __init__(self, path):
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.base_id = self.model.body('base_link').id
        self.root_q = self.model.jnt_qposadr[self.model.joint('robot_root').id]
        self.names = [self.model.joint(int(j)).name for j in self.model.actuator_trnid[:, 0]]
        self.qadr = self.model.jnt_qposadr[self.model.actuator_trnid[:, 0]]
        self.dadr = self.model.jnt_dofadr[self.model.actuator_trnid[:, 0]]
        self.leg_indices = np.array([self.names.index(n) for n in LEG_JOINTS])
        self.arm_names = [f'{s}_joint_link{i}' for s in ('left', 'right') for i in range(1, 8)]
        self.arm_indices = np.array([self.names.index(n) for n in self.arm_names])
        home = dict(zip(LEG_JOINTS, stance_command()))
        home.update(zip(self.arm_names, [*AUBO_HOME_LEFT, *AUBO_HOME_RIGHT]))
        self.home = np.array([home.get(n, 0.) for n in self.names])
        self.kp = np.array([1000. if n in LEG_JOINTS else 120. for n in self.names])
        self.kd = np.array([25. if n in LEG_JOINTS else 4. for n in self.names])
        self.effort = self.model.actuator_ctrlrange[:, 1].copy()
        self.ranges = self.model.jnt_range[self.model.actuator_trnid[:, 0]]
        self.target = self.home.copy()
        # Optional actuator feedforward; disabled in the trained reference plant.
        # Runtime Servo enables it only for the selected upper limb. Physics,
        # masses, contact and effort limits remain unchanged.
        self.bias_compensation_mask = np.zeros(self.model.nu,dtype=bool)
        self.velocity_reference = np.zeros(self.model.nu)
        self.reset()

    def reset(self, height=.56):
        self.bias_compensation_mask[:]=False
        self.velocity_reference[:]=0.
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.root_q:self.root_q+7] = [0., 0., height, 1., 0., 0., 0.]
        self.data.qpos[self.qadr] = self.home
        self.target = self.home.copy()
        mujoco.mj_forward(self.model, self.data)

    def step(self, target=None, enabled=True):
        if target is not None:
            target = np.asarray(target)
            if target.shape != self.home.shape or not np.all(np.isfinite(target)):
                raise ValueError('Invalid joint reference')
            self.target = np.clip(target, self.ranges[:, 0], self.ranges[:, 1])
        torque = self.kp*(self.target-self.data.qpos[self.qadr])+self.kd*(self.velocity_reference-self.data.qvel[self.dadr])
        torque += self.bias_compensation_mask*self.data.qfrc_bias[self.dadr]
        self.data.ctrl[:] = np.clip(torque, -self.effort, self.effort) if enabled else 0.
        mujoco.mj_step(self.model, self.data)

    def body_velocity(self):
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.base_id, velocity, 1)
        return velocity

    def foot_contacts(self):
        result = np.zeros(4)
        ids = [self.model.body(leg+'_foot').id for leg in ('FL','FR','RL','RR')]
        for contact in self.data.contact:
            a, b = self.model.geom_bodyid[contact.geom1], self.model.geom_bodyid[contact.geom2]
            for i, bid in enumerate(ids):
                if (a == bid and b == 0) or (b == bid and a == 0): result[i] = 1.
        return result

    def state(self):
        rotation = self.data.xmat[self.base_id].reshape(3,3)
        return dict(height=float(self.data.xpos[self.base_id,2]), gravity=rotation.T@np.array([0.,0.,-1.]),
                    velocity=self.body_velocity(), contacts=self.foot_contacts())
