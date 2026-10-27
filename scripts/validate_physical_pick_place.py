"""Full free-body pick/carry/place acceptance using the production state machine."""
import sys,os,subprocess
os.environ.setdefault('MUJOCO_GL','egl')
sys.path[:0]=['/ws/src/qda_mujoco','/ws/src/qda_kinematics','/ws/src/qda_locomotion']
import argparse,json,time
from pathlib import Path
import numpy as np
import mujoco
from qda_mujoco.scene_control import SceneController
from qda_mujoco.manipulation import PhysicalManipulation

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',default='/ws/output/grasp/acceptance.json');ap.add_argument('--arm',default='left');ap.add_argument('--fault-test',choices=['drop','cancel']);ap.add_argument('--low-speed',action='store_true');ap.add_argument('--video',help='Record live physics frames, at 10 frames per simulation second');args=ap.parse_args()
    if args.video and (args.fault_test or args.low_speed):ap.error('Video is reserved for the complete task')
    c=SceneController('/ws/output/v2_scene/factory_dynamic.xml','/ws/output/v2/dynamics.xml','/ws/output/v2/runs/b2_transfer_v1/initial_policy.onnx','/ws/output/v2_audit/b2_command_response.json')
    p=c.plant;m,d=p.model,p.data;man=PhysicalManipulation(p)
    target=dict(zip(p.names,p.home));rows=[];started=time.monotonic();result={};collisions={}
    renderer=None;encoder=None;next_frame=[0.];preview_stages=set()
    if args.video:
        path=Path(args.video);path.parent.mkdir(parents=True,exist_ok=True)
        m.vis.global_.offwidth=1280;m.vis.global_.offheight=720
        renderer=mujoco.Renderer(m,height=720,width=1280)
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x720',
            '-framerate','10','-i','-','-an','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    def frame():
        if renderer is None or d.time<next_frame[0]:return
        next_frame[0]+= .1
        camera=mujoco.MjvCamera();camera.azimuth=225.;camera.elevation=-25.
        if man.active and man.stage not in ('open','hover','place_hover'):
            camera.lookat[:]=d.xpos[man.object_id]+[0,0,.08];camera.distance=1.15
        else:
            camera.lookat[:]=d.xpos[p.base_id]+[.25,0,.15];camera.distance=2.7;camera.elevation=-32.;camera.azimuth=135.
        options=mujoco.MjvOption();options.geomgroup[1]=0;options.geomgroup[3]=1
        renderer.update_scene(d,camera=camera,scene_option=options)
        pixels=renderer.render()
        encoder.stdin.write(pixels.tobytes())
        if man.stage in ('close','verify_grasp','carrying','release','placed') and man.stage not in preview_stages:
            from PIL import Image
            Image.fromarray(pixels).save(Path(args.video).with_name('preview-'+man.stage+'.png'))
            preview_stages.add(man.stage)
    def record(label):
        row=dict(label=label,base=d.xpos[p.base_id].tolist(),**man.status());rows.append(row)
        print(json.dumps(row),flush=True)
    def step(command):
        man.update(target);c.step(command,target)
        for contact in d.contact:
            names=[m.body(int(m.geom_bodyid[g])).name for g in (contact.geom1,contact.geom2)]
            if any(n.startswith(args.arm+'_') for n in names) and 'pick_object' not in names:
                pair=' / '.join(sorted(names));collisions[pair]=max(collisions.get(pair,0.),-float(contact.dist))
        if c.fault:raise RuntimeError(c.fault)
        if man.error:raise RuntimeError(man.error)
        frame()
    def hold(seconds):
        until=d.time+seconds
        while d.time<until:step(np.zeros(3))
    def navigate(point):
        until=d.time+100
        while d.time<until:
            r=d.xmat[p.base_id].reshape(3,3);delta=np.asarray(point)-d.xpos[p.base_id,:2]
            yaw=np.arctan2(r[1,0],r[0,0])
            if np.linalg.norm(delta)<.04 and abs(yaw)<.03:break
            command=np.r_[np.clip(.8*(r[:2,:2].T@delta),[-.05,-.08],[.12,.08]),np.clip(-yaw,-.25,.25)]
            step(command)
        else:raise RuntimeError('NAV_TIMEOUT')
        record('waypoint')
    def manipulate(operation,point=None):
        man.begin(operation,args.arm,point)
        previous=None
        until=d.time+60
        while man.active and d.time<until:
            step(np.zeros(3))
            if man.stage!=previous:record(man.stage);previous=man.stage
        if not man.completed:raise RuntimeError(man.error or 'MANIPULATION_TIMEOUT')
    try:
        hold(2.)
        navigate([.20,-.16] if args.arm=='left' else [.20,.48]);hold(2.)
        manipulate('pick');hold(2.)
        if args.low_speed:
            began=d.time;velocities=[]
            while d.time-began<20:
                step(np.array([0.,-.03,0.]))
                if d.time-began>10:velocities.append(p.body_velocity()[4])
            actual=float(np.mean(velocities));record('low_speed_verified')
            result=dict(success=actual<-.015 and man.carrying,test_kind='loaded_low_speed',desired_y=-.03,actual_y=actual,max_slip_m=man.max_slip)
            out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(dict(**result,rows=rows),indent=2))
            print(json.dumps(result),flush=True);return 0 if result['success'] else 1
        if args.fault_test:
            if args.fault_test=='cancel':man.cancel()
            until=d.time+2
            while d.time<until:
                if args.fault_test=='drop':
                    for s in ('left','right'):target[f'{args.arm}_finger_{s}_joint']=0.
                man.update(target);c.step(np.zeros(3),target)
            expected='PAYLOAD_LOST' if args.fault_test=='drop' else 'CANCELLED'
            if man.error!=expected or man.completed:raise RuntimeError('FAULT_NOT_REJECTED_'+str(man.status()))
            if args.fault_test=='cancel' and (not man.carrying or d.xpos[man.object_id,2]<.67):raise RuntimeError('CANCEL_DROPPED_PAYLOAD')
            record('fault_verified');result=dict(success=True,test_kind=args.fault_test,error_detected=expected)
            out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(dict(**result,rows=rows),indent=2))
            print(json.dumps(result),flush=True);return 0
        for waypoint in ([.05,-1.9],[3.7,-1.9],[4.36,-.12] if args.arm=='left' else [4.36,.52]):navigate(waypoint)
        hold(2.);manipulate('place',[5.,.2,.578]);hold(2.);record('final')
        result=dict(success=True,placement_error_m=float(np.linalg.norm(d.xpos[man.object_id,:2]-[5.,.2])))
    except Exception as exc:
        record('failure');result=dict(success=False,error=str(exc))
    result.update(wall_seconds=time.monotonic()-started,physics_seconds=float(d.time),binding=c.binding,rows=rows,arm_collision_penetration_m=collisions)
    if encoder:
        encoder.stdin.close()
        if encoder.wait()!=0:result.update(success=False,error='VIDEO_ENCODING_FAILED')
        renderer.close();result['video']=dict(path=args.video,source='frames rendered during this physical run',fps=10,simulation_speed=1.)
    out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','binding')}),flush=True)
    return 0 if result['success'] else 1
if __name__=='__main__':sys.exit(main())
