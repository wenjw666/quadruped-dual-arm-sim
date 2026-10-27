"""Headless contact experiment; never attaches or overwrites the object's state."""
import sys
from pathlib import Path
sys.path[:0] = ['/ws/src/qda_mujoco', '/ws/src/qda_kinematics', '/ws/src/qda_locomotion']
import argparse, json, time
import numpy as np
import mujoco
from scipy.optimize import least_squares
from qda_mujoco.dynamics import DynamicPlant
from qda_kinematics.kinematics import left_grasp_joints

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',default='/ws/output/grasp/probe.json');ap.add_argument('--rl',action='store_true');ap.add_argument('--inspect',action='store_true');ap.add_argument('--transport',action='store_true');args=ap.parse_args()
    controller=None
    if args.rl:
        from qda_mujoco.scene_control import SceneController
        controller=SceneController('/ws/output/v2_scene/factory_dynamic.xml','/ws/output/v2/dynamics.xml','/ws/output/v2/runs/b2_transfer_v1/initial_policy.onnx','/ws/output/v2_audit/b2_command_response.json')
        p=controller.plant
    else:p=DynamicPlant('/ws/output/v2_scene/factory_dynamic.xml')
    m,d=p.model,p.data;kd=mujoco.MjData(m)
    og=m.geom('pick_object_collision_0').id
    m.geom_priority[og]=1;m.geom_solref[og]=[.004,1.];m.geom_solimp[og]=[.95,.99,.001,.5,2.]
    m.geom_condim[og]=6;m.geom_friction[og]=[.9,.01,.001]
    m.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    m.opt.noslip_iterations=0
    m.opt.cone=mujoco.mjtCone.mjCONE_ELLIPTIC
    for i in range(m.neq):
        name=m.joint(int(m.eq_obj1id[i])).name
        if '_right_finger_tip_joint' in name or '_right_inner_knuckle_joint' in name:
            side=name.split('_')[0];m.eq_obj2id[i]=m.joint(side+'_finger_right_joint').id
    # Episode starts at the documented pick docking pose, with a free base.
    d.qpos[p.root_q:p.root_q+2]=[.14,-.16];mujoco.mj_forward(m,d)
    arm=np.array([p.names.index(f'left_joint_link{i}') for i in range(1,8)])
    fingers=np.array([p.names.index(f'left_finger_{s}_joint') for s in ('left','right')])
    p.bias_compensation_mask[p.arm_indices]=True
    hand=m.body('left_gripper').id;obj=m.body('pick_object').id
    tcp_offset=np.array([0.,0.,.145])
    target=p.home.copy();rows=[]
    def pose(data):
        r=data.xmat[hand].reshape(3,3);return data.xpos[hand]+r@tcp_offset,r
    def record(stage):
        contacts=[]
        for ci,c in enumerate(d.contact):
            if obj in (m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                contacts.append([m.geom(c.geom1).name,m.geom(c.geom2).name,float(c.dist),force.tolist()])
        row=dict(stage=stage,t=float(d.time),object=d.xpos[obj].tolist(),tcp=pose(d)[0].tolist(),base=d.xpos[p.base_id].tolist(),fingers=d.qpos[p.qadr[fingers]].tolist(),contacts=contacts)
        rows.append(row);print(json.dumps(row),flush=True)
    def ik(point,seed):
        kd.qpos[:]=d.qpos
        def fun(q):
            kd.qpos[p.qadr[arm]]=q;mujoco.mj_kinematics(m,kd)
            pos,r=pose(kd)
            return np.r_[8*(pos-point),.6*(r[:,2]-[0,0,-1]),.6*(r[:,0]-[0,1,0]),.005*(q-seed)]
        sol=least_squares(fun,seed,bounds=(p.ranges[arm,0]+.01,p.ranges[arm,1]-.01),max_nfev=100,ftol=1e-6)
        return sol.x,float(np.linalg.norm(fun(sol.x)[:3])/8)
    def run(stage,duration,goal=None,point=None,grip=None):
        start=target.copy();end=start.copy()
        if goal is not None:end[arm]=goal
        if grip is not None:end[fingers]=grip
        n=int(duration/m.opt.timestep)
        for i in range(n):
            a=min(1.,i/max(1,n-1));a=a*a*(3-2*a);target[:]=start+(end-start)*a
            if point is not None and i%25==0:
                q,err=ik(point,target[arm]);end[arm]=q;target[arm]=q
            if controller:controller.step(np.zeros(3),dict(zip(p.names,target)))
            else:p.step(target)
        mujoco.mj_forward(m,d);record(stage)
    if args.inspect:
        for g in ('left_finger_left_collision_0','left_finger_right_collision_0'):
            gi=m.geom(g).id;mid=m.geom_dataid[gi];verts=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
            w=verts@d.geom_xmat[gi].reshape(3,3).T+d.geom_xpos[gi]
            local=(w-d.xpos[hand])@d.xmat[hand].reshape(3,3)
            print(g,local.min(axis=0),local.max(axis=0))
            print('inner',local[np.abs(local[:,0])<.043].min(axis=0),local[np.abs(local[:,0])<.043].max(axis=0))
        return
    run('settle',2.)
    point=d.xpos[obj].copy();base=d.xpos[p.base_id].copy();rot=d.xmat[p.base_id].reshape(3,3)
    seeds=left_grasp_joints(rot.T@(point+np.array([0,0,.15])-base),count=8)
    candidates=[ik(point+[0,0,.15],s) for s in seeds]
    if not candidates:
        rng=np.random.default_rng(4)
        candidates=[ik(point+[0,0,.15],s) for s in rng.uniform(p.ranges[arm,0]+.05,p.ranges[arm,1]-.05,(32,7))]
    q,err=min(candidates,key=lambda x:x[1]);print('IK',q.tolist(),err,flush=True)
    run('hover',5.,goal=q)
    for h in np.linspace(.15,0,11)[1:]:
        q,err=ik(point+[0,0,h],target[arm]);run('descend',.4,goal=q)
    run('close',2.,grip=.52)
    for h in np.linspace(0,.18,13)[1:]:
        q,err=ik(point+[0,0,h],target[arm]);run('lift',.3,goal=q)
    run('hold',2.)
    if args.transport:
        if not controller:raise ValueError('Transport requires RL')
        point=d.xpos[obj].copy()
        carry=d.xpos[p.base_id]+d.xmat[p.base_id].reshape(3,3)@np.array([.42,.32,.28])
        q,err=ik(carry,target[arm]);run('carry_pose',4.,goal=q)
        initial_relative=d.xmat[hand].reshape(3,3).T@(d.xpos[obj]-d.xpos[hand])
        for waypoint in ([.05,-1.9],[3.7,-1.9],[4.36,-.12]):
            started=d.time
            while d.time-started<100:
                r=d.xmat[p.base_id].reshape(3,3);delta=np.array(waypoint)-d.xpos[p.base_id,:2]
                yaw=np.arctan2(r[1,0],r[0,0]);cmd=np.r_[np.clip(.8*(r[:2,:2].T@delta),[-.05,-.08],[.12,.08]),np.clip(-yaw,-.25,.25)]
                if np.linalg.norm(delta)<.025 and abs(yaw)<.03:break
                controller.step(cmd,dict(zip(p.names,target)))
                if int(d.time/m.opt.timestep)%500==0:
                    relative=d.xmat[hand].reshape(3,3).T@(d.xpos[obj]-d.xpos[hand])
                    if np.linalg.norm(relative-initial_relative)>.04:raise RuntimeError('PAYLOAD_SLIP')
                    if controller.fault:raise RuntimeError(controller.fault)
            else:
                record('nav_timeout');print(controller.status(),flush=True);raise RuntimeError('NAV_TIMEOUT')
            record('waypoint')
        run('dock',2.)
        point=np.array([5.,.2,.57]);q,err=ik(point+[0,0,.18],target[arm]);run('place_hover',5.,goal=q)
        shift=0.
    else:shift=.16
    for y in np.linspace(0,shift,12)[1:]:
        q,err=ik(point+[0,y,.18],target[arm]);run('transfer',.3,goal=q)
    for h in np.linspace(.18,0,13)[1:]:
        q,err=ik(point+[0,shift,h],target[arm]);run('place',.3,goal=q)
    run('open',2.,grip=0.)
    q,err=ik(point+[0,shift,.18],target[arm]);run('retreat',3.,goal=q)
    run('settled',2.)
    out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(rows,indent=2))

if __name__=='__main__':main()
