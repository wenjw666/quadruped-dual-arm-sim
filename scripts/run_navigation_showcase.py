"""Actual brain -> Nav2 -> free-base RL tours, with measured arrival evidence."""
import argparse,json,math,time,signal
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as NavPath,OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from qda_interfaces.srv import BrainRequest
from qda_brain.contracts import TARGETS
from qda_kinematics.navigation_examples import PERIMETER_TOUR,MULTI_GOAL_TOUR

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--mode',choices=['perimeter','destinations'],default='perimeter');p.add_argument('--targets',nargs='+');a=p.parse_args()
    root=Path(a.out);root.mkdir(parents=True,exist_ok=False)
    # Keep ROS alive long enough to cancel our owned motion and close the AVI.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    node=rclpy.create_node('navigation_showcase');client=node.create_client(BrainRequest,'/brain/request')
    save=node.create_client(Trigger,'/mapping/save_color')
    config=node.create_client(SetParameters,'/qda_mujoco/set_parameters')
    goals=a.targets or (PERIMETER_TOUR if a.mode=='perimeter' else MULTI_GOAL_TOUR)
    state=dict(pose=None,plan=[],grid=None,front=None,sim=0.,goal=None,index=0,rl={},color={},stage='waiting')
    report=dict(success=False,source='production brain API -> Nav2 -> RL free-base dynamics',localization='simulation_truth',mode=a.mode,goals=goals,arrivals=[],trajectory=[],plans=[],frames=0)
    writer=cv2.VideoWriter(str(root/'live.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(1280,720));last_frame=[None];start=[None]
    events=(root/'events.jsonl').open('w',buffering=1)
    def emit(kind,**kw):
        value=dict(kind=kind,sim=state['sim'],wall=time.time());value.update(kw);events.write(json.dumps(value)+'\n');print(json.dumps(value),flush=True)
    def pose(m):
        q=m.pose.pose.orientation;t=m.pose.pose.position
        state['pose']=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        state['sim']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not report['trajectory'] or state['sim']-report['trajectory'][-1][0]>=.2:report['trajectory'].append([state['sim'],*state['pose']])
    def plan(m):
        state['plan']=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
        report['plans'].append(dict(sim=state['sim'],goal=state['goal'],points=state['plan']))
    def photo(m):
        channels=3;image=np.ndarray((m.height,m.width,channels),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1)).copy()
        if m.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        return image
    def front(m):state['front']=photo(m)
    def follow(m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if last_frame[0] is not None and t-last_frame[0]<.09:return
        if start[0] is None:start[0]=t
        frame=np.full((720,1280,3),22,np.uint8);frame[:540,:960]=cv2.resize(photo(m),(960,540))
        if state['front'] is not None:frame[540:720,:240]=cv2.resize(state['front'],(240,180))
        def text(s,x,y,color=(220,230,230),scale=.53):cv2.putText(frame,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
        text('LIVE MUJOCO / NAV2 / RL',255,568,scale=.65)
        text('Target: '+str(state['goal'])+' | '+state['stage'],255,598)
        text('Simulation: %.1f s | no pose playback'%t,255,625)
        if state['pose'] is not None:
            x,y,yaw=state['pose'];text('Measured base: %.2f, %.2f m | %.0f deg'%(x,y,math.degrees(yaw)),255,650)
        text('Odometry: simulation truth | RGB-D: observed pixels',255,677,scale=.48)
        text('ROUTE / ACTUAL ARRIVALS',974,30,scale=.52)
        # Map is the actual occupancy message; unknown cells remain grey.
        def pix(x,y):return (int(975+(x+2)*30),int(315-(y+3.8)*30))
        grid=state['grid']
        if grid is not None:
            values=np.array(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
            xs=np.arange(300)/30-2;ys=(315-(np.arange(282)+45))/30-3.8
            cols=((xs-grid.info.origin.position.x)/grid.info.resolution).astype(int);rows=((ys-grid.info.origin.position.y)/grid.info.resolution).astype(int)
            sample=values[np.clip(rows,0,grid.info.height-1)[:,None],np.clip(cols,0,grid.info.width-1)[None,:]]
            grey=np.where(sample<0,70,np.where(sample>=50,140,32)).astype(np.uint8)
            frame[45:327,975:1275]=grey[:,:,None]
        points=[pix(*v) for v in state['plan']]
        if len(points)>1:cv2.polylines(frame,[np.array(points)],False,(230,150,50),1)
        points=[pix(*v[1:3]) for v in report['trajectory']]
        if len(points)>1:cv2.polylines(frame,[np.array(points)],False,(65,210,245),2)
        if state['pose'] is not None:cv2.circle(frame,pix(*state['pose'][:2]),5,(80,255,80),-1)
        if state['goal']:cv2.drawMarker(frame,pix(*TARGETS[state['goal']][:2]),(50,100,255),cv2.MARKER_CROSS,14,2)
        text('Blue: Nav2 plan / Gold: actual',974,350,scale=.44)
        for i,name in enumerate(goals):
            mark='OK' if i<len(report['arrivals']) else '>>' if i==state['index'] else '--'
            text(mark+' '+name,974,382+i*25,(110,230,130) if mark=='OK' else (220,220,220),.43)
        text('RGB-D map: '+str(state['color'].get('points',0))+' points',974,690,scale=.42)
        copies=1 if last_frame[0] is None else max(1,round((t-last_frame[0])*10))
        for _ in range(min(copies,100)):writer.write(frame)
        report['frames']+=copies;last_frame[0]=t
        if report['frames']%50<copies:cv2.imwrite(str(root/'latest.jpg'),frame)
    def js(m,key):
        try:state[key]=json.loads(m.data)
        except ValueError:pass
    node.create_subscription(Odometry,'/odom',pose,qos_profile_sensor_data)
    node.create_subscription(NavPath,'/plan',plan,10)
    node.create_subscription(OccupancyGrid,'/map',lambda m:state.update(grid=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Image,'/demo/follow/image_raw',follow,qos_profile_sensor_data)
    node.create_subscription(Image,'/camera/image_raw',front,qos_profile_sensor_data)
    node.create_subscription(String,'/mapping/color_status',lambda m:js(m,'color'),10)
    node.create_subscription(String,'/locomotion/rl_status',lambda m:js(m,'rl'),10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def call(payload):
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('BRAIN_UNAVAILABLE')
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=20)
        if not f.done():raise RuntimeError('BRAIN_TIMEOUT')
        return json.loads(f.result().response_json)
    active=False
    try:
        deadline=time.monotonic()+240
        configured=False
        while time.monotonic()<deadline:
            if not client.wait_for_service(timeout_sec=2):continue
            if not configured:
                if not config.wait_for_service(timeout_sec=2):continue
                parameters=[Parameter(name=k,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=True)) for k in ('render_images','demo_camera','mapping_rgbd')]
                parameters += [Parameter(name=k,value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=v)) for k,v in [('capture_profile','navigation'),('demo_camera_mode','inside')]]
                f=config.call_async(SetParameters.Request(parameters=parameters));rclpy.spin_until_future_complete(node,f,timeout_sec=20)
                if not f.done() or not all(x.successful for x in f.result().results):raise RuntimeError('CAPTURE_CONFIGURATION_FAILED')
                configured=True;report['capture_profile']='navigation: RGB-D and observer at 5 Hz; no wrist-camera rendering'
            snapshot=call({'op':'status'});spin(1)
            if state['pose'] is not None and state['rl'].get('ready') and not state['rl'].get('fault') and report['frames']>0:break
        else:raise RuntimeError('RUNTIME_OR_CAMERA_NOT_READY')
        report['start_pose']=state['pose'].copy()
        for i,target in enumerate(goals):
            state.update(goal=target,index=i,stage='NAVIGATING');tid='nav-demo-'+str(time.time_ns())
            reply=call(dict(op='submit',task_id=tid,scene_version=snapshot['scene_version'],proposal=dict(skill='navigate',object_id='none',target_id=target,arm='none')))
            emit('submitted',target=target,task_id=tid,response=reply)
            if not reply.get('accepted'):raise RuntimeError('NAV_REJECTED')
            active=True;deadline=time.monotonic()+900
            while time.monotonic()<deadline:
                spin(.5);s=call(dict(op='status',task_id=tid));rows=s.get('tasks',[])
                if state['rl'].get('fault'):raise RuntimeError('LOCOMOTION_FAULT:'+str(state['rl']['fault']))
                if rows and rows[0]['status'] not in ('RESERVED','RUNNING'):
                    if rows[0]['status']!='SUCCEEDED':raise RuntimeError(str(rows[0]))
                    active=False;spin(1);break
            else:raise RuntimeError('GOAL_TIMEOUT')
            actual=state['pose'].copy();desired=TARGETS[target]
            distance=math.dist(actual[:2],desired[:2]);yaw=abs(math.atan2(math.sin(actual[2]-desired[2]),math.cos(actual[2]-desired[2])))
            row=dict(target=target,desired=desired,actual=actual,xy_error_m=distance,yaw_error_rad=yaw,sim=state['sim'],task_id=tid)
            if distance>.16 or yaw>.23:raise RuntimeError('MEASURED_ARRIVAL_OUTSIDE_TOLERANCE:'+str(row))
            report['arrivals'].append(row);state['stage']='ARRIVED';emit('arrived',**row);spin(3)
        report['success']=True;state['stage']='ALL GOALS REACHED';spin(3)
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or 'OPERATOR_CANCELLED';emit('failed',error=report['error'])
        if active:
            try:call({'op':'pause'})
            except Exception:pass
    finally:
        try:
            if rclpy.ok() and save.wait_for_service(timeout_sec=5):
                f=save.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=30)
                if f.done():report['map_export']=dict(success=f.result().success,message=f.result().message)
        except Exception as exc:report['map_export']=dict(success=False,error=str(exc))
        writer.release();events.close()
        path=np.array(report['trajectory']);report['travel_m']=float(np.linalg.norm(np.diff(path[:,1:3],axis=0),axis=1).sum()) if len(path)>1 else 0
        report['final_pose']=state['pose'];(root/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('plans','trajectory')}),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if report['success'] else 1
if __name__=='__main__':raise SystemExit(main())
