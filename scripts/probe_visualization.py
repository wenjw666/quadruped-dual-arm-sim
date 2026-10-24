import json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path as RosPath
rclpy.init();n=rclpy.create_node('visualization_probe');seen={}
def cb(topic):
 def receive(m):
  record=seen.setdefault(topic,{'count':0});record['count']+=1
  if hasattr(m,'fields'):record.update(frame=m.header.frame_id,points=m.width*m.height,fields=[x.name for x in m.fields])
  if hasattr(m,'markers'):record.update(markers=len(m.markers),points=sum(len(x.points) for x in m.markers))
  if hasattr(m,'encoding'):record.update(size=[m.width,m.height],encoding=m.encoding)
 return receive
for typ,tp in [(PointCloud2,'/cloud_registered'),(PointCloud2,'/mapping/cloud'),(Image,'/perception/debug'),(Image,'/overview/image_raw'),(MarkerArray,'/viz/grids'),(MarkerArray,'/viz/markers')]:
 n.create_subscription(typ,tp,cb(tp),qos_profile_sensor_data)
start=time.monotonic()
while time.monotonic()-start<15:rclpy.spin_once(n,timeout_sec=.1)
import os
Path(f'/ws/output/dual_backend/visualization_domain{os.getenv("ROS_DOMAIN_ID")}.json').write_text(json.dumps(seen,indent=2))
print(json.dumps(seen,indent=2));n.destroy_node();rclpy.shutdown()
