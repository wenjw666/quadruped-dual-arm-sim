"""Bounded live servo diagnostics, preserving command stamps and measured deltas."""
import json
import argparse
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from qda_interfaces.action import VisualAlign


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--depth',type=float,default=.30);args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('servo_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    rows=[];metrics=[];joints={};modes=[]
    node.create_subscription(JointState,'/joint_states',lambda m:joints.update(zip(m.name,m.position)),10)
    def trajectory(m):
        if len(rows)>=30 or not m.points:return
        rows.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'now':node.get_clock().now().nanoseconds*1e-9,
            'names':m.joint_names,'positions':list(m.points[0].positions),
            'delta':[float(q-joints.get(n,q)) for n,q in zip(m.joint_names,m.points[0].positions)]})
    node.create_subscription(JointTrajectory,'/left_arm_controller/servo_joint_trajectory',trajectory,10)
    node.create_subscription(String,'/brain/servo_metrics',lambda m:metrics.append(json.loads(m.data)),10)
    node.create_subscription(String,'/motion/arm_modes',lambda m:modes.append(json.loads(m.data)),10)
    client=ActionClient(node,VisualAlign,'/visual_align')
    client.wait_for_server(timeout_sec=30)
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    f=client.send_goal_async(VisualAlign.Goal(arm='left',marker_id=17,desired_depth=args.depth,timeout_seconds=150.))
    rclpy.spin_until_future_complete(node,f,timeout_sec=10);handle=f.result();report={}
    if handle and handle.accepted:
        f=handle.get_result_async();rclpy.spin_until_future_complete(node,f,timeout_sec=165)
        if f.done():
            result=f.result().result
            report={'status':f.result().status,'error':result.error_code,'success':result.success,
                    'initial_error_px':result.initial_error_px,'final_error_px':result.final_error_px,'desired_depth_m':args.depth}
        else:handle.cancel_goal_async();report={'error':'PROBE_TIMEOUT'}
    report.update(trajectories=rows,metrics=metrics,modes=modes[-10:])
    Path('/ws/output/v2_brain/servo_probe.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('trajectories','metrics','modes')}),flush=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
