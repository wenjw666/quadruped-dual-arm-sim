"""One actual base command publisher, with bounded manual/task/navigation ownership."""
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CommandMux(Node):
    def __init__(self):
        super().__init__('qda_command_mux')
        self.commands={};self.hold=False;self.fault=False
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/motion/command_owner',10)
        for source in ('nav','task','manual'):
            self.create_subscription(Twist,'/cmd_vel_'+source,lambda msg,s=source:self.receive(s,msg),10)
        self.create_subscription(String,'/locomotion/rl_status',self.on_status,10)
        self.create_service(SetBool,'/motion/hold',self.set_hold)
        self.create_timer(.02,self.tick);self.previous_owner=None;self.last_status=-1.

    def receive(self,source,msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            self.commands.pop(source,None);return
        self.commands[source]=(self.get_clock().now().nanoseconds*1e-9,msg)

    def on_status(self,msg):
        try:self.fault=bool(json.loads(msg.data).get('fault'))
        except (ValueError,TypeError,AttributeError):self.fault=True

    def set_hold(self,request,response):
        self.hold=request.data;self.commands.clear();response.success=True
        response.message='Hold enabled' if self.hold else 'Hold released; waiting for a fresh command'
        return response

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        owner='hold' if self.hold or self.fault else 'idle';command=Twist()
        if owner!='hold':
            for source in ('manual','task','nav'):
                if source in self.commands and 0<=now-self.commands[source][0]<=.25:
                    owner=source;command=self.commands[source][1];break
        self.pub.publish(command)
        if owner!=self.previous_owner or now-self.last_status>=.2:
            self.status.publish(String(data=json.dumps(dict(owner=owner,stamp=now,ttl_seconds=.25))))
            self.previous_owner=owner;self.last_status=now


def main():
    rclpy.init();node=CommandMux()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
