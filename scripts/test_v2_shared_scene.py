"""Verify both Servo monitors actually receive the primary scene's OctoMap."""
import json
import time
from pathlib import Path
import rclpy
from moveit_msgs.msg import PlanningScene


def main():
    rclpy.init();node=rclpy.create_node('servo_shared_scene_acceptance');rows={}
    def scene(name,msg):
        octo=msg.world.octomap.octomap
        if octo.data:rows[name]={'octomap_bytes':len(octo.data),'octomap_frame':msg.world.octomap.header.frame_id or octo.header.frame_id,
            'resolution':octo.resolution,'objects':[o.id for o in msg.world.collision_objects]}
    for name,topic in (('move_group','/monitored_planning_scene'),('left','/servo_left/publish_planning_scene'),('right','/servo_right/publish_planning_scene')):
        node.create_subscription(PlanningScene,topic,lambda msg,n=name:scene(n,msg),10)
    end=time.monotonic()+60
    while len(rows)<3 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    report={'monitors':rows,'passed':len(rows)==3 and all(r['resolution']==.05 and r['octomap_frame']=='odom' for r in rows.values())}
    Path('/ws/output/v2_brain/shared_scene_acceptance.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    node.destroy_node();rclpy.shutdown();assert report['passed'],report


if __name__=='__main__':main()
