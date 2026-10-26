"""One actual base command publisher, with bounded manual/task/navigation ownership."""
import json
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from qda_interfaces.srv import BrainRequest
from .control_lease import BaseLease


class CommandMux(Node):
    def __init__(self):
        super().__init__('qda_command_mux')
        self.commands={};self.hold=False;self.fault=False;self.ready=False;self.state_at=0.
        self.lease=BaseLease();self.last_sim=None;self.clock_advanced_at=time.monotonic()
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/motion/command_owner',10)
        for source in ('nav','task','manual'):
            self.create_subscription(Twist,'/cmd_vel_'+source,lambda msg,s=source:self.receive(s,msg),10)
        self.create_subscription(String,'/locomotion/rl_status',self.on_status,10)
        self.create_service(SetBool,'/motion/hold',self.set_hold)
        self.create_service(BrainRequest,'/motion/lease',self.lease_request)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME));self.previous_owner=None;self.last_status=-1.

    def receive(self,source,msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            self.commands.pop(source,None);return
        self.commands[source]=(self.get_clock().now().nanoseconds*1e-9,time.monotonic(),msg)

    def on_status(self,msg):
        try:
            state=json.loads(msg.data);self.fault=bool(state.get('fault'));self.ready=state.get('ready') is True
            self.state_at=time.monotonic()
        except (ValueError,TypeError,AttributeError):self.fault=True;self.ready=False

    def lease_request(self,request,response):
        try:
            data=json.loads(request.request_json)
            result=self.lease.request(data,time.monotonic())
            if data['op'] in ('acquire','mode','release','reset'):
                self.commands.clear()
                self.hold=not (self.lease.current and self.lease.current['mode']=='nav')
        except Exception as exc:result={'ok':False,'error':str(exc)}
        response.response_json=json.dumps(result);return response

    def set_hold(self,request,response):
        if not request.data and (self.lease.current or self.lease.expired):
            response.success=False;response.message='Active/expired lease requires owner operation';return response
        self.hold=request.data;self.commands.clear();response.success=True
        response.message='Hold enabled' if self.hold else 'Hold released; waiting for a fresh command'
        return response

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        wall=time.monotonic()
        if self.last_sim is None or now!=self.last_sim:
            if self.last_sim is not None and now<self.last_sim:self.commands.clear();self.hold=True
            self.last_sim=now;self.clock_advanced_at=wall
        if self.lease.expire(wall):self.commands.clear();self.hold=True
        reason=('fault' if self.fault else 'state_stale' if wall-self.state_at>5 else 'not_ready' if not self.ready
                else 'clock_stale' if wall-self.clock_advanced_at>5 else 'lease_expired' if self.lease.expired
                else 'hold' if self.hold else '')
        owner='hold' if reason else 'idle';command=Twist()
        if owner!='hold':
            sources=('nav',) if self.lease.current else ('manual','task','nav')
            for source in sources:
                if source in self.commands and 0<=now-self.commands[source][0]<=.25 and wall-self.commands[source][1]<=.25:
                    owner=source;command=self.commands[source][2];break
        self.pub.publish(command)
        if owner!=self.previous_owner or wall-self.last_status>=.2:
            lease=self.lease.current or {}
            self.status.publish(String(data=json.dumps(dict(owner=owner,stamp=now,ttl_seconds=.25,reason=reason,
                lease_owner=lease.get('owner'),lease_mode=lease.get('mode'),lease_expired=self.lease.expired,
                state_age_wall_s=wall-self.state_at))))
            self.previous_owner=owner;self.last_status=wall


def main():
    rclpy.init();node=CommandMux()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
