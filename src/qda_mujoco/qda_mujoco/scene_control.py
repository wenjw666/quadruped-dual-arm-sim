"""Bind the verified robot physics to a different world without hiding model changes."""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from qda_mujoco.dynamics import DynamicPlant


def robot_physics_signature(model,version=2):
    if version not in (1,2):raise ValueError('Unknown physics signature version')
    base=model.body('base_link').id
    bodies={base}
    for bid in range(base+1,model.nbody):
        if int(model.body_parentid[bid]) in bodies:bodies.add(bid)
    records=[]
    for bid in sorted(bodies):
        record={'body':model.body(bid).name}
        for attr in ('body_mass','body_inertia','body_ipos','body_iquat','body_gravcomp'):
            record[attr]=np.asarray(getattr(model,attr)[bid]).tolist()
        if bid!=base:
            record['parent']=model.body(int(model.body_parentid[bid])).name
            record['pos']=model.body_pos[bid].tolist();record['quat']=model.body_quat[bid].tolist()
        records.append(record)
    for jid in range(model.njnt):
        if int(model.jnt_bodyid[jid]) not in bodies:continue
        record={'joint':model.joint(jid).name}
        for attr in ('jnt_type','jnt_pos','jnt_axis','jnt_range','jnt_limited','jnt_stiffness'):
            record[attr]=np.asarray(getattr(model,attr)[jid]).tolist()
        dof=int(model.jnt_dofadr[jid]);count=6 if model.jnt_type[jid]==mujoco.mjtJoint.mjJNT_FREE else 1
        for attr in ('dof_damping','dof_armature','dof_frictionloss','dof_solref','dof_solimp'):
            record[attr]=getattr(model,attr)[dof:dof+count].tolist()
        records.append(record)
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) not in bodies:continue
        if not model.geom_contype[gid] and not model.geom_conaffinity[gid]:continue
        record={'geom':model.geom(gid).name,'body':model.body(int(model.geom_bodyid[gid])).name}
        for attr in ('geom_type','geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','geom_condim','geom_margin','geom_gap'):
            record[attr]=np.asarray(getattr(model,attr)[gid]).tolist()
        if model.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH:
            mid=int(model.geom_dataid[gid]);start=int(model.mesh_vertadr[mid]);count=int(model.mesh_vertnum[mid])
            record['vertices']=hashlib.sha256(model.mesh_vert[start:start+count].tobytes()).hexdigest()
        records.append(record)
    for aid in range(model.nu):
        jid=int(model.actuator_trnid[aid,0])
        if int(model.jnt_bodyid[jid]) in bodies:
            record={'motor':model.joint(jid).name}
            for attr in ('actuator_ctrlrange','actuator_gear','actuator_gainprm','actuator_biasprm','actuator_dynprm',
                         'actuator_gaintype','actuator_biastype','actuator_dyntype','actuator_forcerange'):
                record[attr]=np.asarray(getattr(model,attr)[aid]).tolist()
            records.append(record)
    # Equality constraints and collision exclusions are part of the robot contract.
    records.append({'equalities':model.eq_data.tolist(),'equality_types':model.eq_type.tolist(),
                    'equality_objects':[model.eq_obj1id.tolist(),model.eq_obj2id.tolist()],
                    'equality_solver':[model.eq_solref.tolist(),model.eq_solimp.tolist()],
                    'exclusions':model.exclude_signature.tolist(),'timestep':float(model.opt.timestep),
                    'gravity':model.opt.gravity.tolist(),'integrator':int(model.opt.integrator),
                    'solver':int(model.opt.solver),'iterations':int(model.opt.iterations),
                    'cone':int(model.opt.cone),'enableflags':int(model.opt.enableflags),
                    'disableflags':int(model.opt.disableflags),'noslip_iterations':int(model.opt.noslip_iterations)})
    if version==1:
        for key in ('cone','enableflags','disableflags','noslip_iterations'):records[-1].pop(key)
    return hashlib.sha256(json.dumps(records,sort_keys=True,separators=(',',':')).encode()).hexdigest()


class SceneController:
    def __init__(self,scene_path,reference_path,policy_path,response_path):
        from qda_locomotion.transfer_env import B2TransferEnv
        from qda_locomotion.onnx_policy import OnnxPolicy
        from qda_locomotion.velocity_adapter import VelocityAdapter
        self.env=B2TransferEnv(reference_path,randomize=False)
        self.actor=OnnxPolicy(policy_path,self.env.contract,experimental=True)
        reference_signature=robot_physics_signature(self.env.plant.model)
        self.plant=DynamicPlant(scene_path)
        scene_signature=robot_physics_signature(self.plant.model)
        if scene_signature!=reference_signature:
            # Accept only the explicitly audited right-jaw mimic repair. All
            # other body, collision, actuator and solver differences still fail.
            from .contact_profile import repair_gripper_mimics
            repair_gripper_mimics(self.env.plant.model)
            if scene_signature!=robot_physics_signature(self.env.plant.model):
                raise ValueError(f'Scene robot physics differs from trained plant: {scene_signature} != {reference_signature}')
        if self.plant.names!=self.env.plant.names:raise ValueError('Scene actuator order mismatch')
        old=self.env.plant
        self.plant.home[:]=old.home;self.plant.kp[:]=old.kp;self.plant.kd[:]=old.kd
        self.plant.effort[:]=old.effort
        self.plant.reset()
        self.env.plant=self.plant
        self.env.payload_id=self.plant.model.body('training_payload').id
        self.env.command[:]=0.;self.env.previous_action[:]=0.
        if not self.actor.metadata.get('reference_equivalent'):raise ValueError('Calibration requires original-equivalent actor')
        self.adapter=VelocityAdapter(self.plant,response_path,self.env.contract['model_sha256'])
        self.policy_ticks=0;self.physics_ticks=0;self.fault='';self.estimate={}
        self.target=self.plant.home.copy();self.ready=False
        from .contact_profile import apply_contact_profile
        contact_profile=apply_contact_profile(self.plant.model)
        self.plant.bias_compensation_mask[self.plant.arm_indices]=True
        self.binding=dict(reference_model_sha256=self.env.contract['model_sha256'],
                          scene_model_sha256=hashlib.sha256(Path(scene_path).read_bytes()).hexdigest(),
                          robot_physics_signature=scene_signature,runtime_ready=False,
                          motion='RL_ONNX_free_base_PD',localization='simulation_truth',grasp='physical_contact',
                          contact_profile=contact_profile,physics_signature_version=2,upper_limb_control='bounded_PD_plus_bias',
                          velocity_adapter='measured_breakaway_v2',
                          effective_robot_physics_signature=robot_physics_signature(self.plant.model))
        Path(scene_path).with_suffix('.policy_binding.json').write_text(json.dumps(self.binding,indent=2))

    def step(self,command,upper_targets):
        p=self.plant;s=p.state()
        tilt=float(np.arccos(np.clip(-s['gravity'][2],-1,1)))
        if not np.isfinite(p.data.qpos).all() or s['height']<.30 or tilt>.7:self.fault='FALL_OR_NONFINITE_STATE'
        self.ready=bool(p.data.time>=1. and not self.fault)
        if self.fault:
            p.kp[:]=0.;p.kd[:]=8.;p.step(p.target)
            return
        if p.data.time<1.:
            p.kp[self.env.indices]=1000.;p.kd[self.env.indices]=25.
            p.step(p.home)
            return
        p.kp[self.env.indices]=160.;p.kd[self.env.indices]=5.
        if self.physics_ticks%self.env.decimation==0:
            self.env.command=np.asarray(command,dtype=float)
            obs=self.env.observation()
            obs[6:9],self.estimate=self.adapter.command(command,p)
            action,_=self.actor.predict(obs)
            self.env.previous_action=action.copy()
            self.target=p.home.copy();self.target[self.env.indices]=self.env.q0+self.env.action_scale*action
            self.policy_ticks+=1
        for name,value in upper_targets.items():
            if name in p.arm_names or 'finger' in name:
                if name in p.names:self.target[p.names.index(name)]=value
        p.step(self.target);self.physics_ticks+=1

    def status(self):
        state=self.plant.state()
        return dict(**self.binding,ready=self.ready,fault=self.fault,policy_ticks=self.policy_ticks,
                    physics_ticks=self.physics_ticks,height=state['height'],
                    desired_velocity=self.env.command.tolist(),actual_body_velocity=state['velocity'][[3,4,2]].tolist(),
                    foot_names=['FL','FR','RL','RR'],foot_contacts=state['contacts'].tolist(),
                    foot_positions_world=[self.plant.data.xpos[self.plant.model.body(leg+'_foot').id].tolist() for leg in ('FL','FR','RL','RR')],
                    tilt_rad=float(np.arccos(np.clip(-state['gravity'][2],-1,1))),velocity_estimate=self.estimate)
