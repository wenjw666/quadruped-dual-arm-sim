"""Simulation-only adapter for the pinned rl_sar B2/robot_lab policy.

Contract source: fan-ziqi/rl_sar, revision 376d42c9, policy/b2 and
src/rl_sar/library/core/rl_sdk/rl_sdk.cpp (Ziqi Fan, Apache-2.0).
No ROS command publisher or hardware transport is enabled here.
"""
import hashlib
from pathlib import Path
import numpy as np
import torch
import yaml

EXPECTED = {
    'base.yaml': '144ccfa91383b2d9cdbca25645e80f70d4ce2ac3f66c73c78b7751e6fc818aa6',
    'config.yaml': 'b8ddd49db8c55ff436a4671b3ddf68e8134a8c22ecb9973f18febf426f1accad',
    'policy.pt': 'd4aeb84894acda40f64feadaaad25d2be2ed3b8593b695b148e28673dff21e44',
}


class UpstreamB2Policy:
    def __init__(self, folder, plant):
        folder = Path(folder)
        for name, digest in EXPECTED.items():
            if hashlib.sha256((folder/name).read_bytes()).hexdigest() != digest:
                raise ValueError(f'Unreviewed reference artifact: {name}')
        self.config = yaml.safe_load((folder/'base.yaml').read_text())['b2']
        self.config.update(yaml.safe_load((folder/'config.yaml').read_text())['b2/robot_lab'])
        self.names = self.config['joint_names']
        self.indices = np.array([plant.names.index(name) for name in self.names])
        if len(set(self.indices)) != 12:
            raise ValueError('B2 joint mapping must be a bijection')
        self.q0 = np.array(self.config['default_dof_pos'])
        self.scale = np.array(self.config['action_scale'])
        self.previous = np.zeros(12)
        self.period = self.config['dt'] * self.config['decimation']
        self.decimation = round(self.period / plant.model.opt.timestep)
        if abs(self.decimation * plant.model.opt.timestep - self.period) > 1e-9:
            raise ValueError('Policy period must be an integer number of physics steps')
        self.policy = torch.jit.load(str(folder/'policy.pt'), map_location='cpu').eval()

    def observation(self, plant, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('Invalid velocity command')
        state = plant.state()
        c = self.config
        obs = np.concatenate([
            state['velocity'][:3] * c['ang_vel_scale'], state['gravity'],
            command * np.asarray(c['commands_scale']),
            (plant.data.qpos[plant.qadr[self.indices]] - self.q0) * c['dof_pos_scale'],
            plant.data.qvel[plant.dadr[self.indices]] * c['dof_vel_scale'], self.previous,
        ])
        if obs.shape != (45,) or not np.isfinite(obs).all():
            raise ValueError('Invalid B2 observation')
        return np.clip(obs, -c['clip_obs'], c['clip_obs']).astype(np.float32)

    def target(self, plant, command):
        obs = self.observation(plant, command)
        with torch.inference_mode():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).cpu().numpy()
        if action.shape != (1, 12) or not np.isfinite(action).all():
            raise ValueError('Invalid B2 policy output')
        self.previous = np.clip(action[0], self.config['clip_actions_lower'],
                                self.config['clip_actions_upper'])
        target = plant.home.copy()
        target[self.indices] = self.q0 + self.scale * self.previous
        return target

    def enable_gains(self, plant):
        plant.kp[self.indices] = self.config['rl_kp']
        plant.kd[self.indices] = self.config['rl_kd']
        plant.effort[self.indices] = np.minimum(plant.effort[self.indices], self.config['torque_limits'])
