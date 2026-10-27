"""Read-only ROS evidence recorder. Images are actual sensor frames, never synthesized.

Use a new output directory per take. Motion is launched separately via the normal
mission interface. JSONL records include receive time and source simulation time.
"""
import argparse
import json
import time
import queue
import threading
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=900)
    a=p.parse_args();root=Path(a.out);root.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('product_evidence_recorder');bridge=CvBridge()
    start=time.monotonic();start_unix=time.time();counts={};last={};latest_odom={}
    stream=(root/'events.jsonl').open('w',buffering=1024*1024)
    writes=queue.Queue(maxsize=128);dropped=0;write_errors=[]
    def writer():
        while True:
            item=writes.get()
            try:
                if item is None:return
                if item[0]=='event':stream.write(item[1])
                else:
                    _,name,msg,event=item
                    path=root/name;path.parent.mkdir(exist_ok=True)
                    if item[0]=='map':
                        values=np.array(msg.data).reshape(msg.info.height,msg.info.width)
                        pixels=np.where(values<0,115,255-np.clip(values,0,100)*2.55).astype(np.uint8)
                    else:pixels=bridge.imgmsg_to_cv2(msg,'bgr8')
                    if not cv2.imwrite(str(path),pixels,[cv2.IMWRITE_JPEG_QUALITY,90]):
                        raise OSError('Unable to write '+name)
                    stream.write(event)
            except Exception as exc:write_errors.append(str(exc))
            finally:writes.task_done()
    thread=threading.Thread(target=writer,daemon=True);thread.start()
    def enqueue(item):
        nonlocal dropped
        try:writes.put_nowait(item)
        except queue.Full:dropped+=1
    def entry(kind,data,sim=None):return json.dumps(dict(wall=time.monotonic()-start,sim=sim,kind=kind,data=data),ensure_ascii=False)+'\n'
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def record(kind,data,sim=None):
        enqueue(('event',entry(kind,data,sim)))
    def photo(key,msg):
        t=stamp(msg)
        missing_stamp=t==0 and key=='perception'
        sampling_time=time.monotonic()-start if missing_stamp else t
        if sampling_time-last.get(key,-1)<(.10 if key=='follow' else .20)-1e-6:return
        last[key]=sampling_time;n=counts.get(key,0);counts[key]=n+1
        name=f'{key}/{n:06d}.jpg'
        enqueue(('image',name,msg,entry(key,dict(file=name,timestamp_source='receive_time_missing_header' if missing_stamp else 'header'),None if missing_stamp else t)))
    for key,topic in dict(overview='/overview/image_raw',front='/camera/image_raw',
        follow='/demo/follow/image_raw',c0='/cameras/c0/image_raw',left='/cameras/left_wrist/image_raw',right='/cameras/right_wrist/image_raw',
        servo='/brain/servo_overlay',perception='/perception/debug').items():
        node.create_subscription(Image,topic,lambda m,k=key:photo(k,m),qos_profile_sensor_data)
    node.create_subscription(String,'/perception/labels',lambda m:record('detections',dict(labels=m.data)),10)
    node.create_subscription(PoseStamped,'/perception/pick_pose',lambda m:record('pick_pose',dict(frame=m.header.frame_id,
        xyz=[m.pose.position.x,m.pose.position.y,m.pose.position.z]),stamp(m)),10)
    for key,topic in dict(mission='/brain/mission_status',skill='/brain/status',health='/brain/health',
                         manipulation='/manipulation/status',
                         trace='/demo/brain_trace',decision='/brain/decision_trace',metrics='/brain/servo_metrics',owner='/motion/command_owner',rl='/locomotion/rl_status').items():
        qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL) if key=='trace' else 10
        node.create_subscription(String,topic,lambda m,k=key:record(k,json.loads(m.data)),qos)
    def odom(m):
        q=m.pose.pose.orientation;v=m.pose.pose.position
        latest_odom.update(position=[v.x,v.y,v.z],quaternion_wxyz=[q.w,q.x,q.y,q.z],sim=stamp(m))
    def joints(m):
        t=stamp(m)
        if t-last.get('joints',-1)<.04:return
        last['joints']=t
        record('joints',dict(joints=dict(zip(m.name,m.position)),base=dict(latest_odom)),t)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(NavPath,'/plan',lambda m:record('plan',[[x.pose.position.x,x.pose.position.y] for x in m.poses],stamp(m)),10)
    def grid(msg):
        t=stamp(msg)
        if t-last.get('map',-100)<1.:return
        last['map']=t;n=counts.get('map',0);counts['map']=n+1;name=f'map/{n:06d}.png'
        meta=dict(file=name,resolution=msg.info.resolution,origin=[msg.info.origin.position.x,msg.info.origin.position.y],
                  width=msg.info.width,height=msg.info.height,frame=msg.header.frame_id)
        enqueue(('map',name,msg,entry('map',meta,t)))
    node.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        while time.monotonic()-start<a.seconds:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        writes.put(None);thread.join();stream.close();(root/'manifest.json').write_text(json.dumps(dict(source='actual ROS messages',images=counts,dropped_records=dropped,write_errors=write_errors,
            unix_start=start_unix,wall_seconds=time.monotonic()-start,notes='No commands published. Images are timestamped live simulation frames.'),indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
