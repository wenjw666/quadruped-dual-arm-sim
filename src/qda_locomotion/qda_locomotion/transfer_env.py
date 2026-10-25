"""PPO adaptation of the pinned B2 actor to the articulated dual-arm plant.

The first 45 features and all 12 actions retain the rl_sar contract. Additional
arm/contact/load features are appended; the warm-start actor initially ignores
them. Simulation truth velocity is used only in reward and evaluation.
"""
import hashlib
import json
from pathlib import Path
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
import yaml
from .rl_env import B2Env
from .upstream_b2 import EXPECTED


class B2TransferEnv(B2Env):
    def __init__(self, model_path='/ws/output/v2/dynamics.xml', task='velocity', randomize=True,
                 episode_seconds=10., reference='/ws/output/v2/upstream_b2'):
        super().__init__(model_path, task, randomize, episode_seconds)
        folder = Path(reference)
        for name in ('base.yaml', 'config.yaml'):
            if hashlib.sha256((folder/name).read_bytes()).hexdigest() != EXPECTED[name]:
                raise ValueError('Unreviewed B2 reference configuration')
        cfg = yaml.safe_load((folder/'base.yaml').read_text())['b2']
        cfg.update(yaml.safe_load((folder/'config.yaml').read_text())['b2/robot_lab'])
        self.indices = np.array([self.plant.names.index(n) for n in cfg['joint_names']])
        self.q0 = np.array(cfg['default_dof_pos'])
        self.action_scale = np.array(cfg['action_scale'])
        self.plant.home[self.indices] = self.q0
        self.plant.kp[self.indices] = cfg['rl_kp']
        self.plant.kd[self.indices] = cfg['rl_kd']
        self.plant.effort[self.indices] = np.minimum(self.plant.effort[self.indices], cfg['torque_limits'])
        self.action_space = spaces.Box(-100., 100., (12,), np.float32)
        self.observation_space = spaces.Box(-100., 100., (78,), np.float32)
        self.contract.update(version=2, environment='B2TransferEnv',
            observation_layout=[['body_omega_scaled_025',3],['projected_gravity',3],['command',3],
                ['upstream_joint_delta',12],['upstream_joint_velocity_scaled_005',12],['previous_action',12],
                ['arm_position',14],['arm_velocity_scaled_005',14],['foot_contacts',4],['configured_payload_mass',1]],
            action_joints=cfg['joint_names'], action_scale=self.action_scale.tolist(), q_nominal=self.q0.tolist(),
            action='q_reference = q_nominal + action_scale * clip(action,-100,100); URDF joint limits enforced',
            kp=self.plant.kp.tolist(),kd=self.plant.kd.tolist(), reference_policy_sha256=EXPECTED['policy.pt'],
            initialization='1 second high-gain PD warmup, excluded from reward; upstream gains during rollout',
            reward_version='transfer_v1', command_range=[[-.3,.4],[-.2,.2],[-.5,.5]],
            randomization='upper mass 0.9..1.1, payload 0.001..0.3 kg, COM xy +-5 mm, friction .65..1.0; arms at home')

    def observation(self):
        p = self.plant; s = p.state()
        obs = np.concatenate([s['velocity'][:3]*.25, s['gravity'], self.command,
            p.data.qpos[p.qadr[self.indices]]-self.q0,
            p.data.qvel[p.dadr[self.indices]]*.05, self.previous_action,
            p.data.qpos[p.qadr[p.arm_indices]], p.data.qvel[p.dadr[p.arm_indices]]*.05,
            s['contacts'], [p.model.body_mass[self.payload_id]]])
        if not np.isfinite(obs).all():
            raise FloatingPointError('Invalid transfer observation')
        return np.clip(obs,-100.,100.).astype(np.float32)

    def reset(self, seed=None, options=None):
        _, info = super().reset(seed=seed, options=options)
        p = self.plant
        kp, kd = p.kp.copy(), p.kd.copy()
        p.kp[self.indices] = 1000.; p.kd[self.indices] = 25.
        for _ in range(round(1./p.model.opt.timestep)):
            p.step(p.home)
        p.kp[:], p.kd[:] = kp, kd
        if options and 'command' in options:
            self.command = np.asarray(options['command'], dtype=float)
        elif self.task == 'stand' or self.np_random.random() < .25:
            self.command = np.zeros(3)
        else:
            # Include pure lateral/turn commands rather than training only diagonals.
            self.command = self.np_random.uniform([-.3,-.2,-.5],[.4,.2,.5])
            axis = int(self.np_random.integers(0,4))
            if axis < 3:
                self.command[np.arange(3) != axis] = 0.
        s = p.state()
        if s['height'] < .35 or -s['gravity'][2] < .95:
            raise RuntimeError('Transfer environment warmup failed')
        info['command'] = self.command.tolist()
        return self.observation(), info

    def step(self, action):
        action = np.asarray(action,dtype=float)
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError('Invalid action')
        action = np.clip(action,-100.,100.)
        p = self.plant; target = p.home.copy()
        target[self.indices] = self.q0+self.action_scale*action
        clipped = int(np.any((target < p.ranges[:,0]) | (target > p.ranges[:,1])))
        for _ in range(self.decimation):
            p.step(target)
        self.steps += 1
        s = p.state(); vel = s['velocity']
        xy_error = float(np.linalg.norm(vel[3:5]-self.command[:2]))
        yaw_error = float(abs(vel[2]-self.command[2]))
        tilt = float(np.arccos(np.clip(-s['gravity'][2],-1.,1.)))
        fallen = bool(s['height'] < .30 or s['height'] > .85 or tilt > .7)
        forbidden = 0
        for c in p.data.contact:
            a,b = p.model.geom_bodyid[c.geom1],p.model.geom_bodyid[c.geom2]
            foot_ground = ((a == 0 and p.model.body(b).name.endswith('_foot')) or
                           (b == 0 and p.model.body(a).name.endswith('_foot')))
            if not foot_ground and c.dist < -.002:
                forbidden += 1
        rate = float(np.mean((action-self.previous_action)**2))
        power = float(np.mean(np.abs(p.data.ctrl*p.data.qvel[p.dadr])))
        # Narrow tracking kernels make slow drift distinguishable from standing.
        reward = (3*np.exp(-(xy_error/.12)**2)+1.5*np.exp(-(yaw_error/.25)**2)
                  +.5*np.exp(-(tilt/.15)**2)+.5*np.exp(-((s['height']-.50)/.08)**2)
                  -.02*rate-.0005*power-.5*forbidden-.1*clipped)
        if fallen: reward -= 10.
        self.previous_action = action.copy()
        info = dict(height=s['height'],tilt_rad=tilt,velocity_error_mps=xy_error,yaw_error_rps=yaw_error,
                    body_velocity=vel[3:5].tolist(),body_yaw_rate=float(vel[2]),fallen=fallen,
                    undesired_contacts=forbidden,joint_limit_clip=clipped,sim_s=self.steps*self.dt)
        return self.observation(),float(reward),fallen,bool(self.steps>=self.max_steps),info
