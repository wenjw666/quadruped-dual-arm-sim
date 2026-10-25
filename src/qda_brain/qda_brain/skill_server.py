"""Serial, cancellable ROS skills with a durable idempotency journal."""
import json
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from qda_interfaces.action import RobotSkill, VisualAlign
from qda_interfaces.srv import BrainRequest
from qda_kinematics.kinematics import AUBO_HOME_LEFT, AUBO_HOME_RIGHT, AUBO_SERVO_LEFT, AUBO_SERVO_RIGHT
from .contracts import validate, TARGETS, SCENE_VERSION, Journal


class SkillServer(Node):
    def __init__(self):
        super().__init__('robot_skill_server')
        self.group=ReentrantCallbackGroup();self.lock=threading.RLock()
        self.journal=Journal(self.declare_parameter('journal_path','/ws/output/v2_brain/tasks.sqlite').value)
        self.active=None;self.child=None;self.stop=False;self.cancelled_ids=set();self.state={};self.state_at=0.;self.stable=0
        self.hold=self.create_client(SetBool,'/motion/hold',callback_group=self.group)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.group)
        self.move=ActionClient(self,ExecuteTrajectory,'/execute_trajectory',callback_group=self.group)
        self.align=ActionClient(self,VisualAlign,'/visual_align',callback_group=self.group)
        self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path',callback_group=self.group)
        self.create_subscription(String,'/locomotion/rl_status',self.on_state,10,callback_group=self.group)
        self.create_service(BrainRequest,'/skills/query',self.query,callback_group=self.group)
        self.status_pub=self.create_publisher(String,'/brain/status',10)
        self.create_timer(.5,self.publish_status,callback_group=self.group)
        self.server=ActionServer(self,RobotSkill,'/robot_skill',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=self.cancel,callback_group=self.group)

    def publish_status(self):
        rows=self.journal.get();latest=rows[0] if rows else {}
        self.status_pub.publish(String(data=json.dumps({'active':self.active,'scene_version':SCENE_VERSION,
            'last_skill':latest.get('goal',{}).get('skill','none'),'last_status':latest.get('status','idle'),
            'last_error':latest.get('result',{}).get('error_code',''),'physical_grasp_ready':False})))

    def on_state(self,msg):
        state=json.loads(msg.data);v=state['actual_body_velocity']
        stable=state.get('ready') and not state.get('fault') and math.hypot(*v[:2])<.02 and abs(v[2])<.08 and state['tilt_rad']<.1
        self.stable=self.stable+1 if stable else 0;self.state=state;self.state_at=time.monotonic()

    def query(self,request,response):
        try:
            data=json.loads(request.request_json);op=data.get('op','status')
            if op in ('pause','cancel'):
                self.stop=True
                self.cancelled_ids.update(r['task_id'] for r in self.journal.get() if r['status'] in ('RUNNING','RESERVED'))
                if self.child:self.child.cancel_goal_async()
                if self.hold.service_is_ready():self.hold.call_async(SetBool.Request(data=True))
            elif op!='status':raise ValueError('UNKNOWN_QUERY')
            result={'ok':True,'scene_version':SCENE_VERSION,'active':self.active,
                'tasks':self.journal.get(data.get('task_id')),'locomotion':self.state,
                'state_age_wall_s':time.monotonic()-self.state_at,
                'capabilities':{'navigate':True,'home':True,'visual_align':True,'physical_pick_place':False}}
        except Exception as exc:result={'ok':False,'error':str(exc)}
        response.response_json=json.dumps(result);return response

    def proposal(self,g):return validate({k:getattr(g,k) for k in ('skill','object_id','target_id','arm')})

    def accept(self,g):
        try:
            p=self.proposal(g)
            if g.scene_version!=SCENE_VERSION:raise ValueError('STALE_SCENE')
            # Duplicate terminal tasks return their recorded result without moving again.
            self.journal.reserve(g.task_id,p)
            return GoalResponse.ACCEPT
        except ValueError as exc:
            self.get_logger().warning(str(exc));return GoalResponse.REJECT

    def cancel(self,handle):
        self.cancelled_ids.add(handle.request.task_id)
        if self.active==handle.request.task_id:
            self.stop=True
            if self.child:self.child.cancel_goal_async()
            if self.hold.service_is_ready():self.hold.call_async(SetBool.Request(data=True))
        return CancelResponse.ACCEPT

    def wait(self,future,deadline,check_stop=True):
        while not future.done():
            if check_stop and self.stop:raise RuntimeError('CANCELLED')
            if time.monotonic()>deadline:raise TimeoutError('DEADLINE_EXCEEDED')
            time.sleep(.02)
        return future.result()

    def base_hold(self,value,deadline):
        if not self.hold.wait_for_service(timeout_sec=2):raise RuntimeError('HOLD_SERVICE_UNAVAILABLE')
        result=self.wait(self.hold.call_async(SetBool.Request(data=value)),deadline)
        if not result.success:raise RuntimeError('HOLD_REJECTED')

    def settle(self,deadline):
        self.base_hold(True,deadline)
        while self.stable<5 or time.monotonic()-self.state_at>2:
            if self.stop:raise RuntimeError('CANCELLED')
            if self.state.get('fault'):raise RuntimeError('LOCOMOTION_FAULT')
            if time.monotonic()>deadline:raise TimeoutError('BASE_NOT_STABLE')
            time.sleep(.05)

    def action(self,client,goal,deadline):
        if not client.wait_for_server(timeout_sec=3):raise RuntimeError('SKILL_DEPENDENCY_UNAVAILABLE')
        future=client.send_goal_async(goal)
        # Always obtain acceptance before acting on cancellation, to avoid an orphan goal.
        try:handle=self.wait(future,deadline,False)
        except Exception:
            future.add_done_callback(lambda f:f.result().cancel_goal_async() if f.result() and f.result().accepted else None)
            raise
        if not handle.accepted:raise RuntimeError('CHILD_GOAL_REJECTED')
        self.child=handle
        try:
            if self.stop:raise RuntimeError('CANCELLED')
            result=self.wait(handle.get_result_async(),deadline)
            if result.status!=4:raise RuntimeError(getattr(result.result,'error_code','') or 'CHILD_ACTION_FAILED_'+str(result.status))
            return result.result
        finally:
            if self.stop or time.monotonic()>deadline:handle.cancel_goal_async()
            self.child=None

    def execute(self,handle):
        g=handle.request;record=self.journal.get(g.task_id)[0]
        with self.lock:
            if record['status'] not in ('RESERVED','RUNNING'):
                r=record['result']
                if r.get('success',False):handle.succeed()
                else:handle.abort()
                return RobotSkill.Result(success=r.get('success',False),error_code=r.get('error_code',''),report_json=json.dumps(r))
            if self.active:
                handle.abort();return RobotSkill.Result(success=False,error_code='DUPLICATE_RUNNING',report_json='{}')
            self.active=g.task_id;self.stop=handle.is_cancel_requested or g.task_id in self.cancelled_ids;self.journal.update(g.task_id,'RUNNING')
        deadline=time.monotonic()+(240 if g.skill=='navigate' else 210 if g.skill=='visual_align' else 90);report={'task_id':g.task_id,'skill':g.skill}
        try:
            if self.stop:raise RuntimeError('CANCELLED')
            handle.publish_feedback(RobotSkill.Feedback(stage=g.skill,progress=0.,detail='validated'))
            if g.skill=='inspect':report['locomotion']=self.state
            elif g.skill=='pause':self.base_hold(True,deadline)
            elif g.skill=='pick_place':raise RuntimeError('PHYSICAL_GRASP_NOT_VALIDATED')
            elif g.skill=='navigate':
                if not self.state.get('ready') or self.state.get('fault') or time.monotonic()-self.state_at>2:raise RuntimeError('LOCOMOTION_NOT_READY')
                self.base_hold(False,deadline)
                x,y,yaw=TARGETS[g.target_id];goal=NavigateToPose.Goal()
                goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
                goal.pose.pose.position.x=float(x);goal.pose.pose.position.y=float(y)
                goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
                self.action(self.nav,goal,deadline);self.settle(deadline)
            elif g.skill in ('home','visual_align'):
                self.settle(deadline)
                if not self.plan.wait_for_service(timeout_sec=3):raise RuntimeError('MOVEIT_UNAVAILABLE')
                request=GetMotionPlan.Request();p=request.motion_plan_request;p.group_name=g.arm+'_arm'
                p.planner_id='RRTConnect';p.pipeline_id='ompl';p.allowed_planning_time=5.;p.num_planning_attempts=1
                p.max_velocity_scaling_factor=.15;p.max_acceleration_scaling_factor=.15;p.start_state.is_diff=True
                home=AUBO_HOME_LEFT if g.arm=='left' else AUBO_HOME_RIGHT
                if g.skill=='visual_align':home=AUBO_SERVO_LEFT if g.arm=='left' else AUBO_SERVO_RIGHT
                p.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=f'{g.arm}_joint_link{i+1}',
                    position=float(v),tolerance_above=.01,tolerance_below=.01,weight=1.) for i,v in enumerate(home)])]
                plan=self.wait(self.plan.call_async(request),deadline).motion_plan_response
                if plan.error_code.val!=1:raise RuntimeError('PLANNING_FAILED_'+str(plan.error_code.val))
                result=self.action(self.move,ExecuteTrajectory.Goal(trajectory=plan.trajectory),deadline)
                if result.error_code.val!=1:raise RuntimeError('EXECUTION_FAILED')
                if g.skill=='visual_align':
                    self.settle(deadline)
                    result=self.action(self.align,VisualAlign.Goal(arm=g.arm,marker_id=17,desired_depth=.30,timeout_seconds=150.),deadline)
                    report.update(initial_error_px=result.initial_error_px,final_error_px=result.final_error_px)
                    if not result.success:raise RuntimeError(result.error_code)
            report.update(success=True,error_code='');handle.succeed();status='SUCCEEDED'
        except Exception as exc:
            if self.child:self.child.cancel_goal_async()
            if self.hold.service_is_ready():self.hold.call_async(SetBool.Request(data=True))
            report.update(success=False,error_code=str(exc));status='CANCELLED' if self.stop else 'FAILED'
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
        finally:
            self.journal.update(g.task_id,status,report)
            with self.lock:self.active=None
        return RobotSkill.Result(success=report['success'],error_code=report['error_code'],report_json=json.dumps(report))


def main():
    rclpy.init();node=SkillServer();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
