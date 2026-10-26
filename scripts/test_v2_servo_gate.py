"""Live negative tests for ownership, stale/invalid commands and zero-stamp compatibility."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from qda_interfaces.srv import BrainRequest


def main():
    rclpy.init();node=rclpy.create_node('servo_gate_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={};joints={};report={};owned=False
    node.create_subscription(String,'/motion/arm_modes',lambda m:state.update(json.loads(m.data)),10)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.update(zip(m.name,m.position)),10)
    mode=node.create_client(SetBool,'/left_arm_controller/servo_mode')
    lease=node.create_client(BrainRequest,'/left_arm_controller/servo_lease')
    action=ActionClient(node,FollowJointTrajectory,'/left_arm_controller/follow_joint_trajectory')
    pub=node.create_publisher(JointTrajectory,'/left_arm_controller/servo_joint_trajectory',1)
    names=[f'left_joint_link{i}' for i in range(1,8)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def result(f):
        rclpy.spin_until_future_complete(node,f,timeout_sec=8)
        if not f.done():raise TimeoutError('GATE_TIMEOUT')
        return f.result()
    def count():return state.get('accepted_servo_messages',{}).get('left',0)
    def rejected(reason):return state.get('rejected_servo_messages',{}).get('left',{}).get(reason,0)
    def wait_for(predicate):
        end=time.monotonic()+8
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert predicate(),state
    try:
        assert mode.wait_for_service(timeout_sec=20);spin(3)
        assert state and all(n in joints for n in names)
        reply=result(mode.call_async(SetBool.Request(data=True)));assert reply.success;owned=True
        token=json.loads(reply.message)['token']
        trajectory=JointTrajectory(joint_names=names,points=[JointTrajectoryPoint(positions=[joints[n] for n in names],velocities=[0.]*7,time_from_start=Duration(sec=1))])
        handle=result(action.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory)))
        report['trajectory_rejected_during_servo']=not handle.accepted
        if handle.accepted:handle.cancel_goal_async()
        before=count();previous_rejection=rejected('stale');trajectory.header.stamp.sec=max(1,int(node.get_clock().now().nanoseconds*1e-9)-3)
        pub.publish(trajectory);wait_for(lambda:rejected('stale')>previous_rejection);report['stale_rejected']=count()==before
        trajectory.header.stamp.sec=0;trajectory.header.stamp.nanosec=0
        pub.publish(trajectory);wait_for(lambda:count()>before);report['zero_stamp_accepted']=True
        before=count();previous_rejection=rejected('joint_limit');trajectory.points[0].positions[0]=100.
        pub.publish(trajectory);wait_for(lambda:rejected('joint_limit')>previous_rejection);report['out_of_range_rejected']=count()==before
        assert result(mode.call_async(SetBool.Request(data=False))).success;owned=False
        previous_rejection=rejected('unowned');trajectory.points[0].positions=[joints[n] for n in names];pub.publish(trajectory)
        wait_for(lambda:rejected('unowned')>previous_rejection)
        report['unowned_rejected']=count()==before
        reply=result(mode.call_async(SetBool.Request(data=True)));assert reply.success;owned=True
        new_token=json.loads(reply.message)['token']
        def renew(t):return json.loads(result(lease.call_async(BrainRequest.Request(request_json=json.dumps({'op':'renew','token':t})))).response_json)
        report['old_owner_token_rejected']=not renew(token)['ok']
        end=time.monotonic()+6
        while time.monotonic()<end:assert renew(new_token)['ok'];spin(.5)
        spin(.5)
        report['lease_survives_without_motor_commands']='left' in state.get('servo',[])
        report['motor_command_expires_independently']=state.get('servo_command_fresh',{}).get('left') is False
        assert result(mode.call_async(SetBool.Request(data=False))).success;owned=False
        report['passed']=all(report.values());assert report['passed'],report
    finally:
        if owned:result(mode.call_async(SetBool.Request(data=False)))
        Path('/ws/output/v2_brain/servo_gate_acceptance.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
