"""Read-only smoke test of live backend data, independent of the task log."""
import json,time
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import JointState,Image,LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Bool
rclpy.init();node=rclpy.create_node('backend_topic_smoke');seen={};names=[];ready=False
def joints(m):
    global names
    names=list(m.name);seen['joints']=seen.get('joints',0)+1
def mark(key):
    def callback(m):seen[key]=seen.get(key,0)+1
    return callback
def on_ready(m):
    global ready
    ready=m.data
node.create_subscription(JointState,'/joint_states',joints,10)
node.create_subscription(Odometry,'/odom',mark('odom'),10)
node.create_subscription(Image,'/camera/image_raw',mark('rgb'),qos_profile_sensor_data)
node.create_subscription(Image,'/camera/depth/image_raw',mark('depth'),qos_profile_sensor_data)
node.create_subscription(LaserScan,'/scan',mark('scan'),qos_profile_sensor_data)
node.create_subscription(OccupancyGrid,'/map',mark('map'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(Bool,'/locomotion/ready',on_ready,10)
deadline=time.monotonic()+90
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.2)
    if ready and len(names)>=30 and all(seen.get(k,0)>0 for k in ('odom','rgb','depth','scan','map')):break
ok=ready and len(names)>=30 and all(seen.get(k,0)>0 for k in ('odom','rgb','depth','scan','map'))
print(json.dumps(dict(ready=ready,joint_count=len(names),messages=seen,passed=ok),indent=2),flush=True)
node.destroy_node();rclpy.shutdown();raise SystemExit(0 if ok else 1)
