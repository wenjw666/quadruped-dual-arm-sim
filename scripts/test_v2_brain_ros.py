"""Exercise the live offline brain, replay protection and rendered-pixel servo."""
import json
import argparse
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from qda_interfaces.srv import BrainRequest
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--arm',choices=['left','right'],default='left');args=parser.parse_args()
    home_phrase=('左' if args.arm=='left' else '右')+'臂回位';align_phrase=('左' if args.arm=='left' else '右')+'臂对准'
    rclpy.init();node=rclpy.create_node('brain_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    client=node.create_client(BrainRequest,'/brain/request');report={};frames={};bridge=CvBridge();servo_samples=[]
    def metric(msg):
        row=json.loads(msg.data);row['sim_time']=node.get_clock().now().nanoseconds*1e-9
        if row['arm']==args.arm:servo_samples.append(row)
    node.create_subscription(String,'/brain/servo_metrics',metric,10)
    for side in ('left','right'):
        node.create_subscription(Image,f'/cameras/{side}_wrist/image_raw',lambda m,s=side:frames.update({s:m}),qos_profile_sensor_data)
    def wait(pred,seconds=30):
        end=time.monotonic()+seconds
        while not pred() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        if not pred():raise TimeoutError('ROS_WAIT_TIMEOUT')
    def call(data):
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(data)));wait(f.done,15)
        return json.loads(f.result().response_json)
    def task(proposal,task_id,timeout=100):
        accepted=call(dict(op='submit',task_id=task_id,scene_version=report['scene_version'],proposal=proposal))
        if not accepted.get('accepted'):return accepted
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            state=call(dict(op='status',task_id=task_id));rows=state.get('tasks',[])
            if rows and rows[0]['status'] not in ('RUNNING','RESERVED'):
                print(proposal['skill']+': '+rows[0]['status']+' '+rows[0]['result'].get('error_code',''),flush=True)
                return rows[0]
            rclpy.spin_once(node,timeout_sec=.15)
        call({'op':'cancel'});raise TimeoutError('TASK_TIMEOUT')
    out=Path('/ws/output/v2_brain');prefix='accept-'+str(int(time.time()))
    try:
        wait(client.service_is_ready,60);status=call({'op':'status'});report['scene_version']=status['scene_version']
        report['initial_status']={k:status[k] for k in ('scene_version','active','locomotion','capabilities')}
        for phrase in ('查询状态',home_phrase,align_phrase,'不要去北侧'):
            report[phrase]=call(dict(op='interpret',text=phrase))
        assert report['查询状态']['ok'] and not report['不要去北侧']['ok']
        goal=report['查询状态']['proposal'];report['inspect']=task(goal,prefix+'-inspect')
        report['replay']=task(goal,prefix+'-inspect');assert report['replay']==report['inspect']
        stale=call(dict(op='submit',task_id=prefix+'-stale',scene_version=0,proposal=goal))
        report['stale']=stale;assert not stale['ok']
        wait(lambda:len(frames)==2,30)
        report['markers']={}
        for side,msg in frames.items():
            image=bridge.imgmsg_to_cv2(msg,'bgr8');cv2.imwrite(str(out/(side+'_before.png')),image)
            _,ids,_=cv2.aruco.detectMarkers(image,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
            report['markers'][side]=ids.flatten().tolist() if ids is not None else []
        report['home']=task(report[home_phrase]['proposal'],prefix+'-home')
        report['align']=task(report[align_phrase]['proposal'],prefix+'-align',220)
        if args.arm in frames:cv2.imwrite(str(out/(args.arm+'_after.png')),bridge.imgmsg_to_cv2(frames[args.arm],'bgr8'))
        report['passed']=report['home']['result']['success'] and report['align']['result']['success']
    finally:
        report['servo_samples']=servo_samples
        (out/('ros_acceptance.json' if args.arm=='left' else 'ros_acceptance_right.json')).write_text(json.dumps(report,indent=2,ensure_ascii=False))
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
