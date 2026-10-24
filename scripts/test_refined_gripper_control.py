import json,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
rclpy.init();n=Node('gripper_model_validation');positions={}
n.create_subscription(JointState,'/joint_states',lambda m:positions.update(zip(m.name,m.position)),10)
c=ActionClient(n,FollowJointTrajectory,'/left_gripper_controller/follow_joint_trajectory')
assert c.wait_for_server(timeout_sec=60)
results=[]
for angle in [.42,0.]:
 goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['left_finger_left_joint','left_finger_right_joint']
 goal.trajectory.points=[JointTrajectoryPoint(positions=[angle,angle],time_from_start=Duration(sec=2))]
 f=c.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=20);h=f.result();assert h and h.accepted
 f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=90);assert f.done(), 'Trajectory timed out'
 res=f.result();assert res.result.error_code==0,str(res)
 deadline=time.monotonic()+2
 while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
 observed={k:v for k,v in positions.items() if 'left_' in k and ('finger' in k or 'knuckle' in k)}
 for key in ('left_finger_left_joint','left_finger_right_joint'):assert abs(observed[key]-angle)<.01,(key,observed)
 for key,mult in [('left_robotiq_85_left_inner_knuckle_joint',1),('left_robotiq_85_right_inner_knuckle_joint',-1),('left_robotiq_85_left_finger_tip_joint',-1),('left_robotiq_85_right_finger_tip_joint',1)]:
  # Humble gazebo_ros2_control exports dependent states with a _mimic suffix;
  # robot_state_publisher computes the original URDF mimic names itself.
  actual=observed.get(key,observed.get(key+'_mimic'))
  assert actual is not None and abs(actual-angle*mult)<.015,(key,observed)
 results.append({'command':angle,'observed':observed});print(results[-1],flush=True)
Path(f'/ws/output/model_refinement_v1/gripper_domain{os.getenv("ROS_DOMAIN_ID")}.json').write_text(json.dumps(results,indent=2))
print('GRIPPER_CONTROL_MIMIC_PASS',flush=True);n.destroy_node();rclpy.shutdown()
