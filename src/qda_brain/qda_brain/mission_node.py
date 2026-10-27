"""Durable mission sequence with cancellation acknowledgement between steps."""
import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import SetBool
from qda_interfaces.action import RobotSkill
from qda_interfaces.srv import BrainRequest
from .contracts import Journal
from .mission import validate_plan, child_id


class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        self.group=ReentrantCallbackGroup();self.lock=threading.RLock()
        self.journal=Journal(self.declare_parameter('journal_path','/ws/output/v2_brain/missions.sqlite').value)
        self.client=ActionClient(self,RobotSkill,'/robot_skill',callback_group=self.group)
        self.hold=self.create_client(SetBool,'/motion/hold',callback_group=self.group)
        self.skills=self.create_client(BrainRequest,'/skills/query',callback_group=self.group)
        self.session_token=None;self.renew_future=None;self.renew_at=0.
        self.active=None;self.cancelled=threading.Event();self.child=None;self.quarantine='';self.worker=None
        self.status=self.create_publisher(String,'/brain/mission_status',10)
        self.create_service(BrainRequest,'/mission/request',self.request,callback_group=self.group)
        self.create_timer(.5,self.publish,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.5,self.renew_session,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def renew_session(self):
        with self.lock:
            if self.renew_future is not None and time.monotonic()-self.renew_at>3:
                self.cancelled.set();self.quarantine='SESSION_RENEW_TIMEOUT'
            if not self.session_token or self.renew_future is not None:return
            token=self.session_token
            f=self.skills.call_async(BrainRequest.Request(request_json=json.dumps({'op':'session_renew','token':token})))
            self.renew_future=f;self.renew_at=time.monotonic()
        def done(future):
            with self.lock:
                self.renew_future=None
                if self.session_token!=token:return
                try:ok=json.loads(future.result().response_json).get('ok')
                except Exception:ok=False
                if not ok:self.cancelled.set();self.quarantine='SKILL_SESSION_LOST'
        f.add_done_callback(done)

    def session_request(self,payload):
        if not self.skills.wait_for_service(timeout_sec=3):raise RuntimeError('SKILL_QUERY_UNAVAILABLE')
        future=self.skills.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        try:result=json.loads(self.wait(future,time.monotonic()+3,False).response_json)
        except Exception:
            if payload['op']=='session_acquire':
                def release_late(f):
                    try:
                        r=json.loads(f.result().response_json)
                        if r.get('token'):self.skills.call_async(BrainRequest.Request(request_json=json.dumps({'op':'session_release','token':r['token']})))
                    except Exception:pass
                future.add_done_callback(release_late)
            raise
        if not result.get('ok'):raise RuntimeError(result.get('error','SKILL_SESSION_REJECTED'))
        return result

    def snapshot(self,mission_id=None):
        return dict(ok=True,active=self.active,missions=self.journal.get(mission_id),quarantine=self.quarantine)

    def publish(self):
        rows=self.journal.get();row=rows[0] if rows else {}
        self.status.publish(String(data=json.dumps(dict(active=self.active,mission_id=row.get('task_id'),
            status=row.get('status','IDLE'),**row.get('result',{}),quarantine=self.quarantine))))

    def request(self,request,response):
        try:
            if len(request.request_json)>16384:raise ValueError('REQUEST_TOO_LARGE')
            data=json.loads(request.request_json);op=data.get('op')
            with self.lock:
                if op=='mission_submit':
                    if self.quarantine:raise ValueError('MISSION_QUARANTINED_RESTART_REQUIRED')
                    plan=validate_plan(data['plan']);mid=data['mission_id']
                    reservation=self.journal.reserve(mid,plan)
                    if not reservation['duplicate']:
                        self.active=mid;self.cancelled.clear()
                        self.worker=threading.Thread(target=self.run,args=(mid,plan),daemon=True);self.worker.start()
                    result=dict(ok=True,mission_id=mid,accepted=True,duplicate=reservation['duplicate'],status=reservation['status'])
                elif op=='mission_cancel':
                    mid=data.get('mission_id')
                    if mid and self.active and mid!=self.active:raise ValueError('MISSION_ID_NOT_ACTIVE')
                    if self.active:
                        self.cancelled.set()
                        if self.child:self.child.cancel_goal_async()
                        if self.hold.service_is_ready():self.hold.call_async(SetBool.Request(data=True))
                    result=self.snapshot(mid)
                elif op=='mission_status':result=self.snapshot(data.get('mission_id'))
                else:raise ValueError('UNKNOWN_MISSION_OPERATION')
        except Exception as exc:result=dict(ok=False,error=str(exc))
        response.response_json=json.dumps(result);return response

    def wait(self,future,deadline,cancellable=True):
        while not future.done():
            if cancellable and self.cancelled.is_set():raise RuntimeError('CANCELLED')
            if time.monotonic()>deadline:raise TimeoutError('STEP_DEADLINE_EXCEEDED')
            if not rclpy.ok():raise RuntimeError('SHUTDOWN')
            time.sleep(.02)
        return future.result()

    def run(self,mid,plan):
        report=dict(completed_steps=0,total_steps=len(plan['steps']),step_results=[],current_step=0,stage='starting')
        self.journal.update(mid,'RUNNING',report);state='FAILED'
        try:
            self.session_token=self.session_request({'op':'session_acquire','owner':mid})['token']
            for index,proposal in enumerate(plan['steps']):
                if self.cancelled.is_set():raise RuntimeError('CANCELLED')
                report.update(current_step=index,stage=proposal['skill']);self.journal.update(mid,'RUNNING',report)
                if not self.client.wait_for_server(timeout_sec=3):raise RuntimeError('SKILL_SERVER_UNAVAILABLE')
                future=self.client.send_goal_async(RobotSkill.Goal(task_id=child_id(mid,index),scene_version=plan['scene_version'],**proposal))
                try:handle=self.wait(future,time.monotonic()+8,False)
                except Exception:
                    self.quarantine='LATE_CHILD_ACCEPTANCE'
                    future.add_done_callback(lambda f:f.result().cancel_goal_async() if f.result() and f.result().accepted else None)
                    raise
                if not handle.accepted:raise RuntimeError('SKILL_REJECTED_OR_BUSY')
                with self.lock:self.child=handle
                result_future=handle.get_result_async()
                try:
                    if self.cancelled.is_set():raise RuntimeError('CANCELLED')
                    result=self.wait(result_future,time.monotonic()+(2460 if proposal['skill']=='pick_place' else 660 if proposal['skill']=='navigate' else 420))
                finally:
                    if not result_future.done():
                        handle.cancel_goal_async()
                        try:self.wait(result_future,time.monotonic()+15,False)
                        except Exception:self.quarantine='CHILD_STOP_UNCONFIRMED'
                    with self.lock:self.child=None
                detail=json.loads(result.result.report_json)
                report['step_results'].append(dict(index=index,task_id=child_id(mid,index),action_status=result.status,result=detail))
                if result.status!=4 or not result.result.success:raise RuntimeError(result.result.error_code or 'STEP_FAILED')
                report['completed_steps']=index+1;self.journal.update(mid,'RUNNING',report)
            state='SUCCEEDED';report.update(success=True,error_code='',stage='complete')
        except Exception as exc:
            state='CANCELLED' if self.cancelled.is_set() else 'FAILED'
            report.update(success=False,error_code=str(exc),stage='stopped')
            if self.hold.service_is_ready():self.hold.call_async(SetBool.Request(data=True))
        finally:
            if self.session_token and not self.quarantine:
                with self.lock:token=self.session_token;self.session_token=None
                try:self.session_request({'op':'session_release','token':token})
                except Exception:self.quarantine='SESSION_RELEASE_UNCONFIRMED'
            if self.quarantine:
                state='FAILED';report.update(success=False,error_code=self.quarantine)
            with self.lock:
                self.journal.update(mid,state,report)
                self.active=None
            self.publish()


def main():
    rclpy.init();node=MissionCoordinator();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:
        node.cancelled.set()
        if node.child:node.child.cancel_goal_async()
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
