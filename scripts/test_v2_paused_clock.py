"""The arbiter's lease must expire even when ROS /clock is completely paused."""
import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest
from qda_bringup.command_mux import CommandMux


def main():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);mux=CommandMux();probe=rclpy.create_node('paused_clock_probe')
    executor=SingleThreadedExecutor();executor.add_node(mux);executor.add_node(probe);outputs=[];states=[]
    probe.create_subscription(Twist,'/cmd_vel',lambda m:outputs.append(m.linear.x),10)
    probe.create_subscription(String,'/motion/command_owner',lambda m:states.append(json.loads(m.data)),10)
    clock=probe.create_publisher(Clock,'/clock',10);ready=probe.create_publisher(String,'/locomotion/rl_status',10)
    command=probe.create_publisher(Twist,'/cmd_vel_nav',10);lease=probe.create_client(BrainRequest,'/motion/lease')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    def request(data):
        f=lease.call_async(BrainRequest.Request(request_json=json.dumps(data)));end=time.monotonic()+1
        while not f.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert f.done();return json.loads(f.result().response_json)
    try:
        spin(.5);stamp=Clock();stamp.clock.sec=100;clock.publish(stamp);spin(.1)
        ready.publish(String(data='{"ready":true,"fault":""}'));spin(.05)
        token=request({'op':'acquire','owner':'paused-test'})['token'];request({'op':'mode','token':token,'mode':'nav'})
        msg=Twist();msg.linear.x=.1;command.publish(msg);spin(.05);assert outputs[-1]==.1
        # Commands remain fresh, while no clock or lease heartbeat is sent.
        end=time.monotonic()+3.3
        while time.monotonic()<end:command.publish(msg);spin(.04)
        assert mux.get_clock().now().nanoseconds==100*10**9
        assert outputs[-1]==0. and states[-1]['lease_expired'],states[-1]
        assert not request({'op':'renew','token':token})['ok']
        print('PASS: paused /clock cannot freeze the steady-clock lease watchdog')
    finally:executor.shutdown();mux.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
