"""Free-base B2 + dual-arm locomotion environment, with 50 Hz joint actions.

Simulation ground truth supplies reward/evaluation only. Actor features are ideal
IMU, encoders, contact flags, command, and action history. These ideal sensors
still require noise/latency validation before deployment.
"""
from pathlib import Path
import hashlib
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from qda_mujoco.dynamics import DynamicPlant
from qda_kinematics.kinematics import LEG_JOINTS

OBSERVATIONS = [('body_omega',3),('projected_gravity',3),('velocity_command',3),
                ('leg_position_delta',12),('leg_velocity_scaled',12),('previous_action',12),
                ('arm_position',14),('arm_velocity_scaled',14),('foot_contacts',4),('configured_payload_mass',1)]
OBS_DIM = sum(size for _,size in OBSERVATIONS)


class B2Env(gym.Env):
    metadata = {'render_modes': []}
    def __init__(self, model_path='/ws/output/v2/dynamics.xml', task='stand', randomize=True, episode_seconds=10.):
        if task not in ('stand','velocity'): raise ValueError(task)
        self.plant = DynamicPlant(model_path)
        self.task,self.randomize = task,randomize
        self.dt = .02
        self.decimation = round(self.dt/self.plant.model.opt.timestep)
        if abs(self.decimation*self.plant.model.opt.timestep-self.dt)>1e-9: raise ValueError('Non-integral decimation')
        self.max_steps = round(episode_seconds/self.dt)
        self.action_scale = .25
        self.action_space = spaces.Box(-1.,1.,(12,),np.float32)
        self.observation_space = spaces.Box(-100.,100.,(OBS_DIM,),np.float32)
        self.nominal_mass = self.plant.model.body_mass.copy()
        self.nominal_inertia = self.plant.model.body_inertia.copy()
        self.nominal_com = self.plant.model.body_ipos.copy()
        self.payload_id=self.plant.model.body('training_payload').id
        self.upper_ids = [i for i in range(1,self.plant.model.nbody) if self.plant.model.body(i).name.startswith(('left_','right_'))]
        self.command = np.zeros(3)
        self.previous_action = np.zeros(12)
        self.steps = 0
        self.contract = dict(version=1, observation_layout=[list(x) for x in OBSERVATIONS], observation_dim=OBS_DIM,
                             action_joints=LEG_JOINTS, action_scale=self.action_scale, policy_hz=50,
                             action='q_reference = q_nominal + action_scale * clip(action,-1,1)',
                             q_nominal=self.plant.home[self.plant.leg_indices].tolist(),
                             kp=self.plant.kp.tolist(),kd=self.plant.kd.tolist(),motor_order=self.plant.names,
                             model_sha256=hashlib.sha256(Path(model_path).read_bytes()).hexdigest(),
                             observation_linear_velocity=False, sensor_model='ideal IMU/encoder/contact',
                             runtime_ready=False)

    def observation(self):
        p=self.plant;s=p.state()
        obs=np.concatenate([s['velocity'][:3],s['gravity'],self.command,
                            p.data.qpos[p.qadr[p.leg_indices]]-p.home[p.leg_indices],
                            p.data.qvel[p.dadr[p.leg_indices]]*.05,self.previous_action,
                            p.data.qpos[p.qadr[p.arm_indices]],p.data.qvel[p.dadr[p.arm_indices]]*.05,s['contacts'],
                            [p.model.body_mass[self.payload_id]]])
        if not np.isfinite(obs).all(): raise FloatingPointError('Non-finite observation')
        return np.clip(obs,-100.,100.).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        p=self.plant;m=p.model
        m.body_mass[:]=self.nominal_mass;m.body_inertia[:]=self.nominal_inertia;m.body_ipos[:]=self.nominal_com
        mass_scale=float(self.np_random.uniform(.9,1.1)) if self.randomize else 1.
        m.body_mass[self.upper_ids]*=mass_scale;m.body_inertia[self.upper_ids]*=mass_scale
        load=float(self.np_random.uniform(.001,.3)) if self.randomize else .001
        m.body_mass[self.payload_id]=load;m.body_inertia[self.payload_id]=max(load*.04**2/6,1e-7)
        if self.randomize:m.body_ipos[p.base_id,:2]+=self.np_random.uniform(-.005,.005,2)
        m.geom_friction[:,0]=float(self.np_random.uniform(.65,1.0)) if self.randomize else .8
        mujoco.mj_setConst(m,p.data)
        p.reset();self.steps=0;self.previous_action=np.zeros(12)
        self.command=np.zeros(3) if self.task=='stand' else self.np_random.uniform([-.05,-.08,-.25],[.15,.08,.25])
        if options and 'command' in options:
            self.command=np.asarray(options['command'],dtype=float)
            if self.command.shape!=(3,) or not np.isfinite(self.command).all():raise ValueError('Invalid command')
        if self.randomize:
            p.data.qpos[p.qadr[p.leg_indices]]+=self.np_random.uniform(-.01,.01,12)
            mujoco.mj_forward(m,p.data)
        return self.observation(),{'upper_mass_scale':mass_scale,'command':self.command.tolist()}

    def step(self, action):
        action=np.asarray(action,dtype=float)
        if action.shape!=(12,) or not np.isfinite(action).all():raise ValueError('Invalid action')
        action=np.clip(action,-1,1)
        p=self.plant;target=p.home.copy();target[p.leg_indices]+=self.action_scale*action
        # Reference rate limit stays in the plant's actual actuator path.
        target=np.clip(target,p.target-.14,p.target+.14)
        for _ in range(self.decimation):p.step(target)
        self.steps+=1;s=p.state();velocity=s['velocity'];xy_error=np.linalg.norm(velocity[3:5]-self.command[:2])
        yaw_error=abs(velocity[2]-self.command[2]);tilt=float(np.arccos(np.clip(-s['gravity'][2],-1,1)))
        fallen=s['height']<.30 or s['height']>.85 or tilt>.7
        rate=float(np.mean((action-self.previous_action)**2))
        energy=float(np.mean(np.abs(p.data.ctrl*p.data.qvel[p.dadr])))
        # Contact-foot velocity penalizes sliding, including during zero-speed hold.
        slip=0.
        for i,leg in enumerate(('FL','FR','RL','RR')):
            v=np.zeros(6);mujoco.mj_objectVelocity(p.model,p.data,mujoco.mjtObj.mjOBJ_BODY,p.model.body(leg+'_foot').id,v,0)
            slip+=s['contacts'][i]*float(np.dot(v[3:5],v[3:5]))
        forbidden=0
        for c in p.data.contact:
            a,b=p.model.geom_bodyid[c.geom1],p.model.geom_bodyid[c.geom2]
            foot_ground=(a==0 and p.model.body(b).name.endswith('_foot')) or (b==0 and p.model.body(a).name.endswith('_foot'))
            if not foot_ground and c.dist<-.001:forbidden+=1
        reward=2*np.exp(-xy_error**2/.04)+np.exp(-yaw_error**2/.25)+np.exp(-tilt**2/.1)+np.exp(-(s['height']-.51)**2/.01)-.1*rate-.002*energy-.2*slip-.5*forbidden
        if fallen:reward-=10.
        self.previous_action=action.copy()
        info=dict(height=s['height'],tilt_rad=tilt,velocity_error_mps=float(xy_error),yaw_error_rps=float(yaw_error),
                  fallen=bool(fallen),foot_slip=float(slip),energy_proxy=energy,undesired_contacts=forbidden,sim_s=self.steps*self.dt)
        return self.observation(),float(reward),bool(fallen),bool(self.steps>=self.max_steps),info
