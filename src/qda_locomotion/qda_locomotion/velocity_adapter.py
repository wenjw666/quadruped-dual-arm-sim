"""Measured command feed-forward plus contact-kinematic velocity feedback.

This is an outer velocity adapter for the original RL policy, not PPO training.
The estimator uses joint encoders, body gyro/orientation and foot contacts; free
joint translation velocity is deliberately excluded. Flat non-slipping ground
is an assumption, and contact disagreement is reported rather than hidden.
"""
import json
from pathlib import Path
import mujoco
import numpy as np
from .upstream_b2 import EXPECTED


class ContactVelocityEstimator:
    def __init__(self,plant):
        self.ids=[plant.model.body(leg+'_foot').id for leg in ('FL','FR','RL','RR')]
        self.jacobian=np.zeros((3,plant.model.nv))
        self.velocity=np.zeros(3)
        self.ready=False

    def update(self,plant,body_omega,contact_flags):
        p=plant;rotation=p.data.xmat[p.base_id].reshape(3,3)
        estimates=[]
        for i,bid in enumerate(self.ids):
            if not contact_flags[i]:continue
            mujoco.mj_jacBody(p.model,p.data,self.jacobian,None,bid)
            joint_velocity=rotation.T @ (self.jacobian[:,p.dadr[p.leg_indices]] @ p.data.qvel[p.dadr[p.leg_indices]])
            radius=rotation.T @ (p.data.xpos[bid]-p.data.xpos[p.base_id])
            estimates.append(-joint_velocity-np.cross(body_omega,radius))
        if not estimates:
            return self.velocity.copy(),False,float('inf')
        values=np.asarray(estimates);estimate=np.median(values,axis=0)
        disagreement=float(np.max(np.linalg.norm(values-estimate,axis=1)))
        valid=len(values)>=2 and disagreement<.25 and np.isfinite(estimate).all()
        if valid:
            self.velocity=estimate if not self.ready else .35*estimate+.65*self.velocity
            self.ready=True
        return self.velocity.copy(),valid,disagreement


class VelocityAdapter:
    def __init__(self,plant,response_file,model_sha256,feedback=True):
        report=json.loads(Path(response_file).read_text())
        if report.get('model_sha256')!=model_sha256 or report.get('policy_sha256')!=EXPECTED['policy.pt']:
            raise ValueError('Command calibration belongs to a different model or policy')
        self.curves={};self.integral=np.zeros(3);self.filtered=np.zeros(3)
        self.estimator=ContactVelocityEstimator(plant);self.feedback=feedback
        for axis in range(3):
            for sign in (-1,1):
                rows=[r for r in report['rows'] if r['axis']==axis and sign*r['command']>0 and not r['fallen']
                      and r['mean_actual'] is not None and np.isfinite(r['mean_actual']).all()]
                moving=sorted((sign*r['mean_actual'][axis],abs(r['command'])) for r in rows if sign*r['mean_actual'][axis]>.025)
                stopped=[abs(r['command']) for r in rows if sign*r['mean_actual'][axis]<=.025]
                if not moving:raise ValueError('Missing command-response direction')
                xs=[0.];ys=[max(stopped,default=0.)]
                for x,y in moving:
                    if x>xs[-1] and y>ys[-1]:xs.append(x);ys.append(y)
                self.curves[axis,sign]=(xs,ys)

    def command(self,desired,plant,dt=.02):
        desired=np.asarray(desired,dtype=float)
        if desired.shape!=(3,) or not np.isfinite(desired).all():raise ValueError('Invalid desired velocity')
        state=plant.state()
        velocity,valid,disagreement=self.estimator.update(plant,state['velocity'][:3],state['contacts'])
        estimated=np.array([velocity[0],velocity[1],state['velocity'][2]])
        feed=np.zeros(3)
        for axis,target in enumerate(desired):
            if abs(target)>.01:
                sign=1 if target>0 else -1;xs,ys=self.curves[axis,sign]
                feed[axis]=sign*np.interp(abs(target),xs,ys)
        if np.linalg.norm(desired)<.01:
            self.integral[:]=0.
            command=np.zeros(3)
        else:
            error=desired-estimated
            if valid and self.feedback:
                self.integral=np.clip(self.integral+error*dt,[-.4,-.4,-.6],[.4,.4,.6])
                correction=np.array([.3,.3,.5])*error+np.array([.2,.2,.3])*self.integral
                correction=np.clip(correction,[-.12,-.12,-.3],[.12,.12,.3])
            else:
                correction=np.zeros(3)
            command=np.clip(feed+correction,[-.6,-.6,-.75],[.6,.6,.75])
        # Bound the policy command change, including stand-to-walk transitions.
        self.filtered+=np.clip(command-self.filtered,-1.5*dt,1.5*dt)
        return self.filtered.copy(),dict(estimated_velocity=estimated.tolist(),contact_valid=bool(valid),
                                         contact_disagreement=disagreement if np.isfinite(disagreement) else None)
