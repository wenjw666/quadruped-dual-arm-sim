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


def main():
    rclpy.init();node=rclpy.create_node('servo_gate_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={};joints={};report={};owned=False
    node.create_subscription(String,'/motion/arm_modes',lambda m:state.update(json.loads(m.data)),10)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.update(zip(m.name,m.position)),10)
    mode=node.create_client(SetBool,'/left_arm_controller/servo_mode')
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
    try:
        assert mode.wait_for_service(timeout_sec=20);spin(3)
        assert state and all(n in joints for n in names)
        assert result(mode.call_async(SetBool.Request(data=True))).success;owned=True
        trajectory=JointTrajectory(joint_names=names,points=[JointTrajectoryPoint(positions=[joints[n] for n in names],velocities=[0.]*7,time_from_start=Duration(sec=1))])
        handle=result(action.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory)))
        report['trajectory_rejected_during_servo']=not handle.accepted
        if handle.accepted:handle.cancel_goal_async()
        before=count();trajectory.header.stamp.sec=max(1,int(node.get_clock().now().nanoseconds*1e-9)-3)
        pub.publish(trajectory);spin(.7);report['stale_rejected']=count()==before
        trajectory.header.stamp.sec=0;trajectory.header.stamp.nanosec=0
        pub.publish(trajectory);spin(.7);report['zero_stamp_accepted']=count()>before
        before=count();trajectory.points[0].positions[0]=100.
        pub.publish(trajectory);spin(.5);report['out_of_range_rejected']=count()==before
        assert result(mode.call_async(SetBool.Request(data=False))).success;owned=False
        trajectory.points[0].positions=[joints[n] for n in names];pub.publish(trajectory);spin(.7)
        report['unowned_rejected']=count()==before
        report['passed']=all(report.values());assert report['passed'],report
    finally:
        if owned:result(mode.call_async(SetBool.Request(data=False)))
        Path('/ws/output/v2_brain/servo_gate_acceptance.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
