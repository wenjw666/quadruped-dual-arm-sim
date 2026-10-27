"""Contact-driven manipulation on the live free-base MuJoCo plant.

Only motor targets are written. IK uses a separate MjData; object/free-base qpos,
external forces, equality attachments and collision masks are never modified.
Simulation truth is explicitly used for object pose and grasp verification.
"""
import math
import numpy as np
import mujoco
from scipy.optimize import least_squares


class PhysicalManipulation:
    def __init__(self, plant):
        self.p = plant
        self.m, self.d = plant.model, plant.data
        self.ik_data = mujoco.MjData(self.m)
        self.object_id = self.m.body('pick_object').id
        self.stage = 'idle'; self.error = ''; self.operation = ''; self.arm = 'left'
        self.active = False; self.carrying = False; self.completed = False
        self.queue = []; self.events = []; self.token = None
        self.last_update = -1.; self.loss_since = None
        self.max_slip = 0.; self.max_penetration = 0.
        self.set_arm('left')

    def set_arm(self, arm):
        if arm not in ('left', 'right'): raise ValueError('INVALID_ARM')
        self.arm = arm
        self.indices = np.array([self.p.names.index(f'{arm}_joint_link{i}') for i in range(1,8)])
        self.fingers = np.array([self.p.names.index(f'{arm}_finger_{s}_joint') for s in ('left','right')])
        self.hand = self.m.body(arm+'_gripper').id
        self.pads = [self.m.body(arm+'_finger_'+s).id for s in ('left','right')]

    def tcp(self, data=None):
        data = self.d if data is None else data
        r = data.xmat[self.hand].reshape(3,3)
        return data.xpos[self.hand] + r @ np.array([0.,0.,.145]), r

    def contacts(self):
        forces = [0.,0.]; penetration = 0.; support = []
        for i,c in enumerate(self.d.contact):
            bodies = [int(self.m.geom_bodyid[g]) for g in (c.geom1,c.geom2)]
            if self.object_id not in bodies: continue
            other = bodies[1] if bodies[0] == self.object_id else bodies[0]
            f = np.zeros(6); mujoco.mj_contactForce(self.m,self.d,i,f)
            if other in self.pads:
                forces[self.pads.index(other)] += max(0.,float(f[0]))
                penetration = max(penetration,-float(c.dist))
            elif f[0] > .01:
                support.append(self.m.body(other).name)
        return forces, penetration, sorted(set(support))

    def relative_object(self):
        return self.d.xmat[self.hand].reshape(3,3).T @ (self.d.xpos[self.object_id]-self.d.xpos[self.hand])

    def status(self):
        force, penetration, support = self.contacts()
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.m,self.d,mujoco.mjtObj.mjOBJ_BODY,self.object_id,velocity,0)
        return dict(active=self.active,completed=self.completed,operation=self.operation,stage=self.stage,
                    error=self.error,arm=self.arm,carrying=self.carrying,physics_time=float(self.d.time),
                    object_xyz=self.d.xpos[self.object_id].tolist(),object_quat_wxyz=self.d.xquat[self.object_id].tolist(),tcp_xyz=self.tcp()[0].tolist(),
                    object_speed=float(np.linalg.norm(velocity[3:])),pad_normal_force_n=force,
                    contact_penetration_m=penetration,support_bodies=support,
                    max_slip_m=self.max_slip,max_penetration_m=self.max_penetration,
                    pose_source='MuJoCo simulation truth',attachment='none',events=list(self.events))

    def ik(self, point, seed, strict=True):
        kd=self.ik_data; kd.qpos[:]=self.d.qpos
        # Parallel to the cube's side faces; downward direction alone permits
        # edge pinches which rotate/slip when the arm lifts.
        def residual(q):
            kd.qpos[self.p.qadr[self.indices]]=q
            mujoco.mj_kinematics(self.m,kd)
            pos,r=self.tcp(kd)
            return np.r_[8*(pos-point),.6*(r[:,2]-[0,0,-1]),.6*(r[:,0]-[0,1,0]),.005*(q-seed)]
        lo=self.p.ranges[self.indices,0]+.01; hi=self.p.ranges[self.indices,1]-.01
        fit=least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),max_nfev=100,ftol=1e-6)
        e=residual(fit.x)
        if strict and (np.linalg.norm(e[:3])/8 > .008 or np.linalg.norm(e[3:9])/.6 > .18):
            raise RuntimeError('CARTESIAN_TARGET_UNREACHABLE')
        return fit.x,float(np.linalg.norm(e[:3])/8 + np.linalg.norm(e[3:9]))

    def hover_solution(self, point):
        current=self.d.qpos[self.p.qadr[self.indices]].copy()
        seeds=[current,np.array([-.3473,-3.1836,2.3841,-.5623,.829,.2996,1.5066])]
        rng=np.random.default_rng(4)
        seeds.extend(rng.uniform(self.p.ranges[self.indices,0]+.05,self.p.ranges[self.indices,1]-.05,(24,7)))
        candidates=[]
        for seed in seeds:
            try:q,error=self.ik(point,seed)
            except RuntimeError:continue
            candidates.append((error+.0001*np.linalg.norm(q-current),q))
        if not candidates: raise RuntimeError('NO_GRASP_POSE_AT_CURRENT_DOCK')
        for _,q in sorted(candidates,key=lambda x:x[0]):
            self.ik_data.qpos[:]=self.d.qpos
            free=True
            for alpha in np.linspace(0.,1.,31):
                self.ik_data.qpos[self.p.qadr[self.indices]]=current+(q-current)*alpha
                mujoco.mj_forward(self.m,self.ik_data)
                if self.arm_penetration(self.ik_data)>.003:
                    free=False;break
            if free:return q
        raise RuntimeError('NO_COLLISION_FREE_APPROACH')

    def arm_penetration(self,data):
        worst=0.
        for contact in data.contact:
            bodies=[int(self.m.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]
            if self.object_id in bodies:continue
            if any(self.m.body(b).name.startswith(self.arm+'_') for b in bodies):
                worst=max(worst,-float(contact.dist))
        return worst

    def begin(self, operation, arm, destination=None):
        if self.active: raise RuntimeError('MANIPULATION_BUSY')
        if operation not in ('pick','place'): raise ValueError('INVALID_MANIPULATION_OPERATION')
        if operation=='pick' and self.carrying: raise RuntimeError('ALREADY_CARRYING')
        if operation=='place' and (not self.carrying or arm!=self.arm): raise RuntimeError('NO_VERIFIED_PAYLOAD')
        self.set_arm(arm)
        point=self.d.xpos[self.object_id].copy() if operation=='pick' else np.asarray(destination,dtype=float)
        if point.shape!=(3,) or not np.isfinite(point).all(): raise ValueError('INVALID_DESTINATION')
        q=self.hover_solution(point+[0,0,.15])
        self.operation=operation; self.completed=False; self.error=''; self.events=[]
        self.pick_z=float(point[2]); self.destination=point.copy()
        if operation=='pick':
            self.max_slip=0.; self.max_penetration=0.; self.loss_since=None
            self.queue=[('open',1.,0.),('hover',5.,q),('descend',5.,point),
                        ('close',2.,.52),('lift',6.,point+[0,0,.18]),('verify_grasp',1.,None)]
        else:
            self.queue=[('place_hover',5.,q),('lower',6.,point),('release',2.,0.),
                        ('retreat',5.,point+[0,0,.18]),('verify_place',2.,None)]
        self.p.bias_compensation_mask[self.p.arm_indices]=True
        self.active=True; self.stage='starting'; self.next_stage()

    def next_stage(self):
        if not self.queue:
            self.active=False; self.completed=True; self.stage='carrying' if self.carrying else 'placed'; return
        self.stage,self.duration,self.goal=self.queue.pop(0)
        self.started=float(self.d.time);self.start_q=self.d.qpos[self.p.qadr[self.indices]].copy()
        self.start_f=self.d.qpos[self.p.qadr[self.fingers]].copy()
        self.start_tcp=self.tcp()[0].copy();self.last_update=-1.
        if self.stage=='release': self.carrying=False
        self.events.append(dict(stage=self.stage,physics_time=float(self.d.time)))

    def fail(self, reason):
        self.error=reason; self.active=False; self.completed=False; self.stage='failed';self.queue=[]
        self.events.append(dict(stage='failed',error=reason,physics_time=float(self.d.time)))

    def cancel(self):
        # Keep the gripping target: cancellation must not deliberately drop a load.
        # Late cleanup must not overwrite a completed placement or the original
        # physical failure with CANCELLED.
        if self.error or (self.completed and not self.active and not self.carrying):return
        self.fail('CANCELLED')

    def update(self, targets):
        force,penetration,support=self.contacts()
        self.max_penetration=max(self.max_penetration,penetration)
        if self.carrying:
            if self.arm_penetration(self.d)>.003:
                self.fail('CARRY_ARM_COLLISION');return
            slip=float(np.linalg.norm(self.relative_object()-self.grasp_relative))
            self.max_slip=max(self.max_slip,slip)
            lost=slip>.035 or min(force)<.05
            if lost and self.loss_since is None:self.loss_since=float(self.d.time)
            if not lost:self.loss_since=None
            if self.loss_since is not None and self.d.time-self.loss_since>.2:
                self.carrying=False;self.fail('PAYLOAD_LOST');return
        if not self.active:return
        try:
            if self.arm_penetration(self.d)>.003:raise RuntimeError('ARM_COLLISION')
            if penetration>.003:raise RuntimeError('EXCESSIVE_GRASP_PENETRATION')
            elapsed=float(self.d.time)-self.started
            alpha=np.clip(elapsed/self.duration,0.,1.);smooth=alpha*alpha*(3-2*alpha)
            if self.stage in ('open','close','release'):
                values=self.start_f+(float(self.goal)-self.start_f)*smooth
                targets.update(zip([self.p.names[i] for i in self.fingers],map(float,values)))
            elif self.stage in ('hover','place_hover'):
                q=self.start_q+(self.goal-self.start_q)*smooth
                targets.update(zip([self.p.names[i] for i in self.indices],map(float,q)))
            elif self.stage in ('descend','lift','lower','retreat') and self.d.time-self.last_update>=.05:
                point=self.start_tcp+(self.goal-self.start_tcp)*smooth
                seed=np.array([targets[self.p.names[i]] for i in self.indices])
                q,_=self.ik(point,seed)
                targets.update(zip([self.p.names[i] for i in self.indices],map(float,q)))
                self.last_update=float(self.d.time)
            if elapsed < self.duration:return
            if self.stage in ('descend','lift','lower','retreat') and np.linalg.norm(self.tcp()[0]-self.goal)>.012:
                if elapsed>self.duration+3:raise RuntimeError('TCP_TRACKING_ERROR')
                return
            if self.stage=='close' and min(force)<.1:raise RuntimeError('NO_BILATERAL_GRASP_CONTACT')
            if self.stage=='verify_grasp':
                if min(force)<.1 or self.d.xpos[self.object_id,2]<self.pick_z+.10 or support:
                    raise RuntimeError('LIFT_NOT_VERIFIED')
                self.grasp_relative=self.relative_object().copy();self.carrying=True
            if self.stage=='verify_place':
                s=self.status();error=float(np.linalg.norm(self.d.xpos[self.object_id,:2]-self.destination[:2]))
                if error>.035 or abs(self.d.xpos[self.object_id,2]-self.destination[2])>.012 or s['object_speed']>.015 or not set(support).intersection(('place_table','place_marker')) or max(force)>.05:
                    raise RuntimeError('PLACEMENT_NOT_STABLE_ON_TARGET')
            self.next_stage()
        except Exception as exc:self.fail(str(exc))
