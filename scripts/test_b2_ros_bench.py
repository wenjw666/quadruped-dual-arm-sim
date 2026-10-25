"""Exercise actual ROS topics, ONNX inference, physics, and command expiration."""
import json
import argparse
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from qda_locomotion.rl_bench_node import RlBench
from qda_locomotion.onnx_policy import OnnxPolicy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--calibrated',action='store_true')
    options=parser.parse_args()
    model='/ws/output/v2/runs/b2_transfer_v1/'+('initial_policy.onnx' if options.calibrated else 'policy.onnx')
    args=['--ros-args','-p',f'policy_path:={model}','-p','experimental:=true','-p','max_seconds:=5.0']
    if options.calibrated:args+=['-p','command_response:=/ws/output/v2_audit/b2_command_response.json']
    rclpy.init(args=args)
    bench=RlBench();observer=rclpy.create_node('b2_bench_observer',namespace='/v2_test')
    states=[];joints=[]
    observer.create_subscription(String,'/v2/rl_status',lambda msg:states.append(json.loads(msg.data)),10)
    observer.create_subscription(JointState,'/v2/joint_states',lambda msg:joints.append(msg),10)
    publisher=observer.create_publisher(Twist,'/v2/cmd_vel',10)
    executor=SingleThreadedExecutor();executor.add_node(bench);executor.add_node(observer)
    start=time.monotonic();last_publish=0.
    try:
        try:OnnxPolicy(model,bench.env.contract)
        except ValueError:pass
        else:raise AssertionError('Unvalidated policy enabled without experimental flag')
        while not bench.finished and time.monotonic()-start<12:
            elapsed=time.monotonic()-start
            if .4<elapsed<2. and elapsed-last_publish>.05:
                command=Twist();command.linear.x=.15;publisher.publish(command);last_publish=elapsed
            executor.spin_once(timeout_sec=.01)
        assert bench.finished,'Bench exceeded wall-time bound'
        assert len(joints)>100,'Missing actual joint stream'
        assert all(len(msg.name)==30 and len(msg.effort)==30 for msg in joints)
        assert any(not s['command_stale'] and s['command'][0]>.1 for s in states),'Command never reached controller'
        assert states[-1]['command_stale'] and states[-1]['command']==[0.,0.,0.],'Watchdog did not clear expired command'
        assert not any(s['fault'] for s in states),states
        result=dict(joint_messages=len(joints),status_messages=len(states),command_received=True,
                    calibrated=options.calibrated,
                    watchdog_stopped_command=True,production_gate_rejected_unvalidated=True,
                    fault='',runtime_ready=False)
        suffix='_calibrated' if options.calibrated else ''
        Path(f'/ws/output/v2_audit/b2_ros_bench_test{suffix}.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    finally:
        executor.shutdown();bench.env.close();bench.destroy_node();observer.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
