"""ROS transport check in an isolated domain, including TTL and hold release."""
import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from qda_bringup.command_mux import CommandMux


def main():
    rclpy.init();mux=CommandMux();probe=rclpy.create_node('mux_probe')
    executor=SingleThreadedExecutor();executor.add_node(mux);executor.add_node(probe)
    outputs=[]
    probe.create_subscription(Twist,'/cmd_vel',lambda m:outputs.append(m.linear.x),10)
    pubs={s:probe.create_publisher(Twist,'/cmd_vel_'+s,10) for s in ('nav','task','manual')}
    fault=probe.create_publisher(String,'/locomotion/rl_status',10)
    hold=probe.create_client(SetBool,'/motion/hold')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    def send(source,value):
        msg=Twist();msg.linear.x=value;pubs[source].publish(msg)
    def service(value):
        future=hold.call_async(SetBool.Request(data=value))
        end=time.monotonic()+1
        while not future.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert future.done() and future.result().success
    try:
        spin(.5)
        send('nav',.1);send('task',.2);send('manual',.3);spin(.08)
        assert outputs[-1]==.3,outputs
        spin(.3);assert outputs[-1]==0.,'Expired commands survived'
        send('nav',.1);send('task',.2);spin(.08);assert outputs[-1]==.2
        service(True);send('manual',.3);spin(.08);assert outputs[-1]==0.
        service(False);spin(.08);assert outputs[-1]==0.,'Hold release replayed stale command'
        send('nav',.1);spin(.08);assert outputs[-1]==.1
        fault.publish(String(data=json.dumps({'fault':'fall'})));spin(.08);assert outputs[-1]==0.
        print('PASS: manual/task/nav priority, expiration, hold, fresh-command release, fault stop')
    finally:
        executor.shutdown();mux.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
