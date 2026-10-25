"""Bounded sim-only transfer probe: original B2 policy on the current dual-arm robot."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
import torch
import xml.etree.ElementTree as ET
from qda_mujoco.dynamics import DynamicPlant
from qda_locomotion.upstream_b2 import UpstreamB2Policy, EXPECTED


class OfficialB2Plant(DynamicPlant):
    """Same PD interface, for the official model which has no separate foot bodies."""
    def __init__(self, path):
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.base_id = self.model.body('base_link').id
        self.root_q = self.model.jnt_qposadr[self.model.joint('floating_base_joint').id]
        self.names = [self.model.joint(int(j)).name for j in self.model.actuator_trnid[:,0]]
        self.qadr = self.model.jnt_qposadr[self.model.actuator_trnid[:,0]]
        self.dadr = self.model.jnt_dofadr[self.model.actuator_trnid[:,0]]
        self.home = np.zeros(12)
        self.kp = np.full(12, 1000.); self.kd = np.full(12, 25.)
        self.effort = self.model.actuator_ctrlrange[:,1].copy()
        self.ranges = self.model.jnt_range[self.model.actuator_trnid[:,0]]
        self.target = self.home.copy()

    def foot_contacts(self):
        # The 45-dimensional reference actor does not consume foot contacts.
        return np.zeros(4)


def build_official_probe(source, output):
    """Remove only massless visual meshes, retaining official inertia/collisions."""
    root = ET.parse(source).getroot()
    for parent in root.iter():
        for geom in list(parent.findall('geom')):
            if geom.get('mesh'):
                if geom.get('contype') != '0' or geom.get('conaffinity') != '0' or geom.get('density') != '0':
                    raise ValueError('Refusing to remove a mesh with physical effects')
                parent.remove(geom)
    root.remove(root.find('asset'))
    ET.SubElement(root, 'option', timestep='.002', gravity='0 0 -9.81')
    world = root.find('worldbody')
    ET.SubElement(world, 'geom', name='ground', type='plane', size='20 20 .1', friction='.8 .005 .0001')
    # Identify the original sphere contacts for the diagnostic only.
    for body in world.iter('body'):
        for geom in body.findall('geom'):
            if geom.get('type') == 'sphere' and body.get('name', '').endswith('_calf'):
                geom.set('name', body.get('name').replace('_calf', '_foot_contact'))
    ET.ElementTree(root).write(output, encoding='unicode')


def evaluate(model, folder, seconds=10., seeds=(7, 8, 9), official=False, speed_profile='low'):
    torch.set_num_threads(2)
    results = []
    commands = [('stand', [0., 0., 0.]), ('forward', [.15, 0., 0.]),
                ('lateral', [0., .10, 0.]), ('turn', [0., 0., .25])]
    if speed_profile == 'moderate':
        commands = [('forward', [.4, 0., 0.]), ('lateral', [0., .25, 0.]), ('turn', [0., 0., .5])]
    for seed in seeds:
        for name, command in commands:
            p = OfficialB2Plant(model) if official else DynamicPlant(model)
            policy = UpstreamB2Policy(folder, p)
            p.home[policy.indices] = policy.q0
            p.reset()
            rng = np.random.default_rng(seed)
            p.data.qpos[p.qadr[policy.indices]] += rng.uniform(-.005, .005, 12)
            mujoco.mj_forward(p.model, p.data)
            # Common deterministic initialization; not counted as RL performance.
            for _ in range(round(2. / p.model.opt.timestep)):
                p.step(p.home)
            warm = p.state()
            warm_tilt = float(np.arccos(np.clip(-warm['gravity'][2], -1., 1.)))
            if warm['height'] < .30 or warm_tilt > .35 or not np.isfinite(p.data.qpos).all():
                raise RuntimeError('Initialization failed before RL; do not score as policy failure')
            policy.enable_gains(p)
            start_xy = p.data.qpos[p.root_q:p.root_q+2].copy()
            errors, yaw_errors, heights, tilts = [], [], [], []
            clipped, saturated, bad_contact_samples = 0, 0, 0
            fallen = False
            for step in range(round(seconds/policy.period)):
                target = policy.target(p, command)
                clipped += int(np.any((target < p.ranges[:,0]) | (target > p.ranges[:,1])))
                for _ in range(policy.decimation):
                    p.step(target)
                    saturated += int(np.any(np.abs(p.data.ctrl[policy.indices]) >= p.effort[policy.indices] - 1e-6))
                s = p.state()
                tilt = float(np.arccos(np.clip(-s['gravity'][2], -1., 1.)))
                heights.append(s['height']); tilts.append(tilt)
                errors.append(float(np.linalg.norm(s['velocity'][3:5] - command[:2])))
                yaw_errors.append(float(abs(s['velocity'][2] - command[2])))
                for contact in p.data.contact:
                    a, b = p.model.geom_bodyid[contact.geom1], p.model.geom_bodyid[contact.geom2]
                    if official:
                        foot_ground = ((a == 0 and (p.model.geom(contact.geom2).name or '').endswith('_foot_contact')) or
                                       (b == 0 and (p.model.geom(contact.geom1).name or '').endswith('_foot_contact')))
                    else:
                        foot_ground = ((a == 0 and p.model.body(b).name.endswith('_foot')) or
                                       (b == 0 and p.model.body(a).name.endswith('_foot')))
                    if not foot_ground and contact.dist < -.002:
                        bad_contact_samples += 1
                        break
                fallen = bool(s['height'] < .30 or tilt > .7 or not np.isfinite(p.data.qpos).all())
                if fallen:
                    break
            result = dict(seed=seed, command_name=name, command=command, seconds=(step+1)*policy.period,
                          initial_height=warm['height'], initial_tilt_rad=warm_tilt,
                          fallen=fallen, min_height=min(heights), max_tilt_rad=max(tilts),
                          mean_velocity_error_mps=float(np.mean(errors)), mean_yaw_error_rps=float(np.mean(yaw_errors)),
                          displacement_xy=(p.data.qpos[p.root_q:p.root_q+2]-start_xy).tolist(),
                          joint_limit_clip_frames=clipped, torque_saturation_steps=saturated,
                          undesired_contact_frames=bad_contact_samples)
            results.append(result)
            print(json.dumps(result), flush=True)
    return dict(model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest(), official_bare_model=official,
                speed_profile=speed_profile, robot_mass_kg=float(p.model.body_subtreemass[p.base_id]),
                policy_sha256=EXPECTED['policy.pt'], episodes=results, runtime_ready=False,
                description='Original bare-B2 policy; no loaded-robot retraining',
                initialization='2 seconds project PD hold at upstream q0; gain switch to upstream 160/5',
                limits='Ideal sensors, flat ground, joint-limit clipping, 2 ms physics / 50 Hz policy; not hardware acceptance')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='/ws/output/v2/dynamics.xml')
    parser.add_argument('--policy', default='/ws/output/v2/upstream_b2')
    parser.add_argument('--output', type=Path, default=Path('/ws/output/v2_audit/b2_reference_evaluation.json'))
    parser.add_argument('--official-source', type=Path, help='Official b2.xml; removes visual meshes only')
    parser.add_argument('--speed-profile', choices=['low','moderate'], default='low')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.official_source:
        args.model = args.output.with_suffix('.model.xml')
        build_official_probe(args.official_source, args.model)
    result = evaluate(args.model, args.policy, official=bool(args.official_source), speed_profile=args.speed_profile)
    if args.official_source:
        result['source_sha256'] = hashlib.sha256(args.official_source.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2))
