"""Read-only, backend-independent evidence for an end-to-end comparison run."""
import argparse,json,time,math
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry,Path as NavPath,OccupancyGrid
from sensor_msgs.msg import Image,JointState,LaserScan,PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--duration',type=float,default=600)
a=parser.parse_args();rclpy.init();node=rclpy.create_node('backend_comparison_monitor')
counts=Counter();state={};start=time.monotonic();last=0.;terminal=False
out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def callback(topic):
 def on(m):
  global terminal
  counts[topic]+=1
  if topic=='/odom':
   state['sim']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
   p=m.pose.pose.position;state['base']=[p.x,p.y,p.z]
  elif topic=='/task/state':
   state['task']=m.data
   terminal=terminal or 'TASK_SUCCESS' in m.data or 'TASK_FAILED' in m.data
  elif topic=='/scan':state['scan_hits']=sum(math.isfinite(x) for x in m.ranges)
  elif topic=='/map':state['occupied_cells']=sum(x>50 for x in m.data)
  elif topic=='/perception/pick_pose' or topic=='/perception/place_pose':
   p=m.pose.position;state[topic]=[p.x,p.y,p.z]
  elif topic=='/perception/labels':state['labels']=m.data
  elif topic.endswith('path') or topic=='/viz/global_plan':state[topic]=len(m.poses)
  elif topic=='/livox/lidar_PointCloud2':state['lidar_points']=m.width*m.height
  elif topic=='/viz/markers':state['dashboard']=m.markers[-1].text if m.markers else ''
 return on
for cls,tp,qos in [(Odometry,'/odom',10),(String,'/task/state',10),(JointState,'/joint_states',10),(LaserScan,'/scan',qos_profile_sensor_data),(PointCloud2,'/livox/lidar_PointCloud2',qos_profile_sensor_data),(PointCloud2,'/mapping/colored',qos_profile_sensor_data),(Image,'/camera/image_raw',qos_profile_sensor_data),(Image,'/camera/depth/image_raw',qos_profile_sensor_data),(Image,'/perception/debug',10),(String,'/perception/labels',10),(PoseStamped,'/perception/pick_pose',10),(PoseStamped,'/perception/place_pose',10),(OccupancyGrid,'/map',latched),(NavPath,'/viz/global_plan',latched),(NavPath,'/viz/actual_path',latched),(NavPath,'/viz/tool_path',latched),(NavPath,'/viz/planned_tool_path',latched),(MarkerArray,'/trajectories',10),(MarkerArray,'/viz/markers',latched)]:
 node.create_subscription(cls,tp,callback(tp),qos)
with out.open('w') as f:
 while rclpy.ok() and time.monotonic()-start<a.duration:
  rclpy.spin_once(node,timeout_sec=.05)
  elapsed=time.monotonic()-start
  if elapsed-last>1:
   f.write(json.dumps({'wall':elapsed,**state,'messages':dict(counts)},ensure_ascii=False)+'\n');f.flush();last=elapsed
  if terminal:break
summary={'wall':time.monotonic()-start,**state,'messages':dict(counts)}
out.with_suffix('.summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False,indent=2));node.destroy_node();rclpy.shutdown()
