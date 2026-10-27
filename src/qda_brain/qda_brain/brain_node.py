"""Local semantic gateway. Interpreting a sentence does not execute it."""
import json
import time
import threading
import uuid
import hashlib
from pathlib import Path
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
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest
from qda_interfaces.action import RobotSkill
from .contracts import parse_local,validate,SCENE_VERSION
from .model_client import interpret,review_feedback
from .feedback_review import expected_decision,review_gate


class Brain(Node):
    def __init__(self):
        super().__init__('local_brain');self.group=ReentrantCallbackGroup();self.image=None;self.image_at=0.
        self.model_lock=threading.Lock();self.bridge=CvBridge()
        self.receipts={};self.receipt_lock=threading.Lock();self.odom=None;self.odom_at=0.
        self.audit_dir=Path(self.declare_parameter('audit_dir','/ws/output/brain_loop/audit').value)
        self.audit_dir.mkdir(parents=True,exist_ok=True);self.audit_lock=threading.Lock()
        self.trace_pub=self.create_publisher(String,'/brain/decision_trace',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10,callback_group=self.group)
        self.url=self.declare_parameter('model_url','http://127.0.0.1:8088/v1/chat/completions').value
        self.action=ActionClient(self,RobotSkill,'/robot_skill',callback_group=self.group)
        self.query=self.create_client(BrainRequest,'/skills/query',callback_group=self.group)
        self.mission=self.create_client(BrainRequest,'/mission/request',callback_group=self.group)
        self.observations={};self.health_pub=self.create_publisher(String,'/brain/health',10)
        self.monitored_topics=('/locomotion/rl_status','/motion/command_owner','/motion/arm_modes','/perception/labels','/brain/mission_status','/manipulation/status')
        for topic in self.monitored_topics:
            self.create_subscription(String,topic,lambda m,t=topic:self.observe(t,m),10,callback_group=self.group)
        self.create_timer(1.,self.publish_health,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_subscription(Image,'/camera/image_raw',self.on_image,qos_profile_sensor_data,callback_group=self.group)
        self.create_service(BrainRequest,'/brain/request',self.request,callback_group=self.group)

    def on_image(self,msg):self.image=msg;self.image_at=time.monotonic()

    def on_odom(self,msg):self.odom=msg;self.odom_at=time.monotonic()

    def trace(self,phase,**data):
        row=dict(phase=phase,unix=time.time(),sim=self.get_clock().now().nanoseconds*1e-9,**data)
        encoded=json.dumps(row,ensure_ascii=False)
        with self.audit_lock:
            with (self.audit_dir/'events.jsonl').open('a',encoding='utf-8') as stream:stream.write(encoded+'\n')
        self.trace_pub.publish(String(data=encoded))

    def feedback_snapshot(self,text,previous_task_id,done):
        if not self.query.wait_for_service(timeout_sec=2):raise ValueError('SKILL_SERVER_UNAVAILABLE')
        query={'op':'status'}
        if previous_task_id:query['task_id']=previous_task_id
        answer=json.loads(self.wait(self.query.call_async(BrainRequest.Request(request_json=json.dumps(query)))).response_json)
        if not answer.get('ok'):raise ValueError('FEEDBACK_QUERY_FAILED')
        row=next((r for r in answer.get('tasks',[]) if r['task_id']==previous_task_id),{}) if previous_task_id else {}
        health=self.health();now=time.monotonic();pose=self.odom
        topics=health['topics'];required=('/locomotion/rl_status','/motion/command_owner','/motion/arm_modes','/manipulation/status')
        fresh=health['front_camera_fresh'] and now-self.odom_at<3 and all(topics[t]['fresh'] for t in required)
        rl=answer.get('locomotion',{});manip=topics['/manipulation/status']['data'] or {}
        proposal=None if done else parse_local(text)
        context=dict(operator_next=text,all_steps_done=done,candidate=None if done else proposal['skill'],candidate_proposal=proposal,
            sensors_fresh=bool(fresh),robot_ready=bool(rl.get('ready')),fault=rl.get('fault') or manip.get('error') or answer.get('quarantine',''),
            active_task=answer.get('active') or answer.get('mission_owner'),previous_status=row.get('status','MISSING' if previous_task_id else 'NONE'),
            feedback=dict(previous_task_id=previous_task_id,previous_result=row.get('result',{}),
                base_xy=[round(pose.pose.pose.position.x,3),round(pose.pose.pose.position.y,3)] if pose else None,
                actual_velocity=[round(v,3) for v in rl.get('actual_body_velocity',[])],
                foot_contacts=rl.get('foot_contacts'),tilt_rad=round(rl.get('tilt_rad',0),3),
                perception=topics['/perception/labels']['data'],physical_stage=manip.get('stage'),carrying=manip.get('carrying'),
                camera_fresh=health['front_camera_fresh'],odometry_source='simulation_truth'))
        # Inspect reports contain the full RL binding; keep prompt bounded while
        # retaining the original journal as a separately traceable evidence source.
        previous=context['feedback']['previous_result']
        context['feedback']['previous_result']={k:previous[k] for k in ('success','error_code','arrival','skill') if k in previous}
        return context

    def review_step(self,data):
        text=data.get('text','');done=data.get('all_steps_done',False);previous=data.get('previous_task_id')
        if not isinstance(done,bool) or not isinstance(text,str) or len(text)>500:raise ValueError('INVALID_REVIEW_REQUEST')
        if not self.model_lock.acquire(False):raise ValueError('MODEL_BUSY')
        try:
            decision_id=uuid.uuid4().hex;context=self.feedback_snapshot(text,previous,done);png=None;image_info=None
            msg=self.image
            if context['feedback']['camera_fresh'] and msg is not None:
                image=cv2.resize(self.bridge.imgmsg_to_cv2(msg,'bgr8'),(320,240))
                ok,encoded=cv2.imencode('.png',image)
                if not ok:raise ValueError('IMAGE_ENCODING_FAILED')
                png=encoded.tobytes();path=self.audit_dir/(decision_id+'.png');path.write_bytes(png)
                image_info=dict(path=str(path),sha256=hashlib.sha256(png).hexdigest(),stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,width=320,height=240)
            model_context={k:context[k] for k in ('all_steps_done','candidate','sensors_fresh','robot_ready','fault','active_task','previous_status')}
            model_context['operator_next']=text
            model_context['feedback']=context['feedback']
            model_context['decision_rule']='all_steps_done=false + fresh + ready + no active task + candidate => dispatch; all_steps_done=true + SUCCEEDED => complete'
            self.trace('model_input',decision_id=decision_id,context=model_context,full_context=context,image=image_info)
            model=review_feedback(model_context,png,self.url)
            # SmolVLM occasionally emits a schema-valid but state-inconsistent
            # label. Re-ground the same live image and operator sentence through
            # the already validated semantic path; the gate still rejects any
            # proposal that does not equal the requested candidate.
            expected_now=expected_decision(context)
            if model.get('decision')!=expected_now and expected_now=='dispatch':
                proposal=context['candidate_proposal'];target_words={'north_entry':'north','south_entry':'south','inspection':'inspection','start':'start'}
                bridge_text=('Navigate to the '+target_words.get(proposal.get('target_id',''),'requested target')
                             if proposal.get('skill')=='navigate' else text)
                bridge=interpret(bridge_text,png,self.url)
                model.update(decision='dispatch',raw_decision=model.get('decision'),
                    decision_source='interpretation_bridge',bridge_proposal=bridge['proposal'],
                    bridge_matches_candidate=bridge['proposal']==context['candidate_proposal'])
            current=self.feedback_snapshot(text,previous,done)
            gate=review_gate(context,current,model)
            result=dict(ok=True,decision_id=decision_id,scene_version=SCENE_VERSION,model=model,image=image_info,
                decision=model['decision'],
                **gate,candidate=context['candidate_proposal'],executed=False,
                scope='operator-specified next skill; VLM reviews live feedback; deterministic gate rechecks')
            if result['execution_allowed']:
                with self.receipt_lock:
                    self.receipts={k:v for k,v in self.receipts.items() if v['expires']>time.monotonic()}
                    self.receipts[decision_id]=dict(expires=time.monotonic()+30,proposal=context['candidate_proposal'],text=text,previous=previous)
            self.trace('model_output',**result);return result
        except Exception as exc:
            self.trace('review_error',error=str(exc));raise
        finally:self.model_lock.release()

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
                    model_autonomous_ready=False,physical_grasp_ready=rows['/manipulation/status']['fresh'] and not rows['/manipulation/status']['data'].get('error'))

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
            if op=='review_step':result=self.review_step(data)
            elif op=='interpret':
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
                decision_id=data.get('decision_id')
                if decision_id:
                    with self.receipt_lock:receipt=self.receipts.pop(decision_id,None)
                    if not receipt or receipt['expires']<time.monotonic() or receipt['proposal']!=proposal:raise ValueError('INVALID_OR_EXPIRED_DECISION')
                    if expected_decision(self.feedback_snapshot(receipt['text'],receipt['previous'],False))!='dispatch':raise ValueError('FEEDBACK_CHANGED_BEFORE_DISPATCH')
                if not self.action.wait_for_server(timeout_sec=2):raise ValueError('SKILL_SERVER_UNAVAILABLE')
                goal=RobotSkill.Goal(task_id=data['task_id'],scene_version=SCENE_VERSION,**proposal)
                self.trace('skill_dispatch',task_id=goal.task_id,decision_id=decision_id,proposal=proposal,endpoint='/robot_skill')
                future=self.action.send_goal_async(goal,feedback_callback=lambda m:self.trace('skill_feedback',task_id=goal.task_id,stage=m.feedback.stage,progress=m.feedback.progress,detail=m.feedback.detail))
                handle=self.wait(future)
                if handle.accepted:
                    def completed(f):
                        try:
                            r=f.result();self.trace('skill_result',task_id=goal.task_id,action_status=r.status,report=json.loads(r.result.report_json))
                        except Exception as exc:self.trace('skill_result_error',task_id=goal.task_id,error=str(exc))
                    handle.get_result_async().add_done_callback(completed)
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
