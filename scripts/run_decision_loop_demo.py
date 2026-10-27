"""Record a real VLM feedback loop: observe -> review -> dispatch -> feedback -> review."""
import argparse,json,time,hashlib
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest
from qda_brain.contracts import SCENE_VERSION

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--text',default='前往南侧入口');a=p.parse_args()
    root=Path(a.out);root.mkdir(parents=True,exist_ok=False);(root/'events.jsonl').touch()
    rclpy.init();node=rclpy.create_node('decision_loop_demo');client=node.create_client(BrainRequest,'/brain/request')
    rows=[];latest={'decision':None,'status':None,'image':None};start=time.monotonic()
    trace_pub=node.create_publisher(String,'/demo/brain_trace',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def record(kind,value):
        row=dict(wall=time.monotonic()-start,kind=kind,data=value);rows.append(row)
        with (root/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        phase={'operator_input':'input','review_response':'proposal','dispatch_response':'submitted',
               'task_terminal':'execution','completion_review':'finished','error':'failed'}.get(kind)
        if phase:
            if phase=='input':payload=dict(phase=phase,unix=time.time(),sentences=[value.get('text','')],use_vlm=True)
            elif phase=='proposal':payload=dict(phase=phase,unix=time.time(),index=0,text=value.get('text',''),response=value)
            elif phase=='submitted':payload=dict(phase=phase,unix=time.time(),response=value)
            elif phase=='execution':payload=dict(phase=phase,unix=time.time(),mission=value)
            else:payload=dict(phase=phase,unix=time.time(),**value)
            trace_pub.publish(String(data=json.dumps(payload,ensure_ascii=False)))
        print(json.dumps(row,ensure_ascii=False),flush=True)
    def image(msg):
        latest['image']=dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,encoding=msg.encoding,width=msg.width,height=msg.height)
    def decision(msg):
        try:value=json.loads(msg.data);latest['decision']=value;record('decision_trace',value)
        except ValueError:record('decision_trace_raw',msg.data)
    def status(msg):
        try:value=json.loads(msg.data);latest['status']=value;record('mission_status',value)
        except ValueError:pass
    node.create_subscription(Image,'/camera/image_raw',image,qos_profile_sensor_data)
    node.create_subscription(String,'/brain/decision_trace',decision,10)
    node.create_subscription(String,'/brain/mission_status',status,10)
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def call(payload,timeout=180):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('BRAIN_UNAVAILABLE')
        future=client.call_async(BrainRequest.Request(request_json=json.dumps(payload,ensure_ascii=False)))
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('BRAIN_TIMEOUT')
        return json.loads(future.result().response_json)
    report=dict(success=False,source='live VLM review and production brain dispatch',text=a.text,events=rows,model_scope='bounded decision review, not autonomous free-form planning')
    try:
        spin(3);record('operator_input',dict(text=a.text,image_available=latest['image'] is not None,scene_version=SCENE_VERSION))
        review=call(dict(op='review_step',text=a.text,all_steps_done=False));record('review_response',review)
        # A wait is a real closed-loop state: acquire a newer observation and
        # ask the model again. No command is sent during this interval.
        for retry in range(3):
            if review.get('execution_allowed'):break
            if review.get('guard_expected')!='wait':raise RuntimeError('VLM_REVIEW_GATE_REJECTED:'+str(review))
            record('controller_wait',dict(retry=retry+1,reason='feedback_not_fresh_or_robot_not_ready'));spin(5)
            review=call(dict(op='review_step',text=a.text,all_steps_done=False));record('review_response',review)
        if not review.get('execution_allowed'):raise RuntimeError('VLM_REVIEW_WAIT_TIMEOUT:'+str(review))
        proposal=review['candidate'];task_id='decision-loop-'+str(time.time_ns())
        submitted=call(dict(op='submit',task_id=task_id,scene_version=SCENE_VERSION,proposal=proposal,decision_id=review['decision_id']));record('dispatch_response',submitted)
        if not submitted.get('accepted'):raise RuntimeError('SKILL_REJECTED')
        deadline=time.monotonic()+700;terminal=None
        while time.monotonic()<deadline:
            spin(.4);state=call(dict(op='status',task_id=task_id));tasks=state.get('tasks',[])
            if tasks and tasks[0]['status'] not in ('RUNNING','RESERVED'):
                terminal=tasks[0];record('task_terminal',terminal);break
        if terminal is None:raise TimeoutError('TASK_TIMEOUT')
        if terminal['status']!='SUCCEEDED':raise RuntimeError('TASK_FAILED:'+str(terminal))
        done=call(dict(op='review_step',text=a.text,previous_task_id=task_id,all_steps_done=True));record('completion_review',done)
        report.update(success=bool(done.get('ok') and done.get('decision')=='complete' and done.get('gate_passed')
            and done.get('guard_current')=='complete'),task_id=task_id,review=review,completion_review=done)
    except Exception as exc:
        report['error']=str(exc);record('error',dict(error=str(exc)))
        try:call({'op':'pause'},timeout=20)
        except Exception:pass
    finally:
        report['events']=rows;report['wall_seconds']=time.monotonic()-start
        (root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if report['success'] else 1
if __name__=='__main__':raise SystemExit(main())
