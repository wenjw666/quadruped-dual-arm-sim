"""Local semantic gateway. Interpreting a sentence does not execute it."""
import json
import time
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest
from qda_interfaces.action import RobotSkill
from .contracts import parse_local,validate,SCENE_VERSION
from .model_client import interpret


class Brain(Node):
    def __init__(self):
        super().__init__('local_brain');self.group=ReentrantCallbackGroup();self.image=None;self.image_at=0.
        self.model_lock=threading.Lock();self.bridge=CvBridge()
        self.url=self.declare_parameter('model_url','http://127.0.0.1:8088/v1/chat/completions').value
        self.action=ActionClient(self,RobotSkill,'/robot_skill',callback_group=self.group)
        self.query=self.create_client(BrainRequest,'/skills/query',callback_group=self.group)
        self.mission=self.create_client(BrainRequest,'/mission/request',callback_group=self.group)
        self.observations={};self.health_pub=self.create_publisher(String,'/brain/health',10)
        self.monitored_topics=('/locomotion/rl_status','/motion/command_owner','/motion/arm_modes','/perception/labels','/brain/mission_status')
        for topic in self.monitored_topics:
            self.create_subscription(String,topic,lambda m,t=topic:self.observe(t,m),10,callback_group=self.group)
        self.create_timer(1.,self.publish_health,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_subscription(Image,'/camera/image_raw',self.on_image,qos_profile_sensor_data,callback_group=self.group)
        self.create_service(BrainRequest,'/brain/request',self.request,callback_group=self.group)

    def on_image(self,msg):self.image=msg;self.image_at=time.monotonic()

    def observe(self,topic,msg):
        try:value=json.loads(msg.data)
        except ValueError:value={'text':msg.data[:256]}
        self.observations[topic]=(time.monotonic(),value)

    def health(self):
        now=time.monotonic();observed=self.observations.copy();rows={}
        for topic in self.monitored_topics:
            at,value=observed.get(topic,(None,None))
            rows[topic]=dict(age_wall_s=round(now-at,3) if at is not None else None,fresh=at is not None and now-at<5,data=value)
        return dict(topics=rows,skill_server=self.action.server_is_ready(),mission_server=self.mission.service_is_ready(),
                    front_camera_fresh=self.image is not None and now-self.image_at<5,
                    model_autonomous_ready=False,physical_grasp_ready=False)

    def publish_health(self):self.health_pub.publish(String(data=json.dumps(self.health())))

    def wait(self,future,seconds=5):
        deadline=time.monotonic()+seconds
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('SERVICE_TIMEOUT')
            time.sleep(.01)
        return future.result()

    def request(self,request,response):
        try:
            if len(request.request_json)>16384:raise ValueError('REQUEST_TOO_LARGE')
            data=json.loads(request.request_json);op=data.get('op','status')
            if op=='interpret':
                text=data.get('text','')
                if not isinstance(text,str) or not 0<len(text)<=500:raise ValueError('INVALID_TEXT')
                if any(w in text.lower() for w in ('不要','别','not','ignore','忽略','然后',';','；')):raise ValueError('AMBIGUOUS_OR_NEGATED_REQUEST')
                if data.get('use_vlm',False):
                    if not self.model_lock.acquire(False):raise ValueError('MODEL_BUSY')
                    try:
                        png=None
                        if data.get('with_image',False):
                            msg=self.image
                            if msg is None or time.monotonic()-self.image_at>2:raise ValueError('STALE_IMAGE')
                            ok,encoded=cv2.imencode('.png',self.bridge.imgmsg_to_cv2(msg,'bgr8'))
                            if not ok:raise ValueError('IMAGE_ENCODING_FAILED')
                            png=encoded.tobytes()
                        result=interpret(text,png,self.url)
                        try:expected=parse_local(text)
                        except ValueError:expected=None
                        result['execution_allowed']=expected is not None and result['proposal']==expected
                        result['semantic_gate']='local_rules_agree' if result['execution_allowed'] else 'model_requires_review'
                    finally:self.model_lock.release()
                else:result={'proposal':parse_local(text),'source':'bounded_local_rules','runtime_ready':False,'execution_allowed':True}
                result.update(ok=True,scene_version=SCENE_VERSION,executed=False)
            elif op=='submit':
                proposal=validate(data['proposal'])
                if data.get('scene_version')!=SCENE_VERSION:raise ValueError('STALE_SCENE')
                if not self.action.wait_for_server(timeout_sec=2):raise ValueError('SKILL_SERVER_UNAVAILABLE')
                goal=RobotSkill.Goal(task_id=data['task_id'],scene_version=SCENE_VERSION,**proposal)
                future=self.action.send_goal_async(goal)
                handle=self.wait(future)
                result={'ok':handle.accepted,'task_id':goal.task_id,'accepted':handle.accepted,
                        'completed':False,'error':'' if handle.accepted else 'SKILL_REJECTED'}
            elif op in ('mission_submit','mission_status','mission_cancel'):
                if not self.mission.wait_for_service(timeout_sec=2):raise ValueError('MISSION_SERVER_UNAVAILABLE')
                result=json.loads(self.wait(self.mission.call_async(BrainRequest.Request(request_json=json.dumps(data)))).response_json)
            elif op in ('status','pause','cancel'):
                if op in ('pause','cancel') and self.mission.service_is_ready():
                    self.wait(self.mission.call_async(BrainRequest.Request(request_json=json.dumps({'op':'mission_cancel'}))))
                if not self.query.wait_for_service(timeout_sec=2):raise ValueError('SKILL_SERVER_UNAVAILABLE')
                answer=self.wait(self.query.call_async(BrainRequest.Request(request_json=json.dumps(data))))
                result=json.loads(answer.response_json)
                result['health']=self.health()
            else:raise ValueError('UNKNOWN_OPERATION')
        except Exception as exc:result={'ok':False,'error':str(exc)}
        response.response_json=json.dumps(result,ensure_ascii=False);return response


def main():
    rclpy.init();node=Brain();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
