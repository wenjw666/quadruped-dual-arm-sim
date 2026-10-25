"""Isolated ROS 2 locomotion bench driven by ONNX and MuJoCo dynamics.

Publishes /v2 state only. No Nav2, hardware transport, virtual grasp or root pose
prescription. An unaccepted policy requires the explicit experimental parameter.
"""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from .transfer_env import B2TransferEnv
from .onnx_policy import OnnxPolicy
from .velocity_adapter import VelocityAdapter


def stamp(seconds):
    ns=round(seconds*1e9)
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)


class RlBench(Node):
    def __init__(self):
        super().__init__('b2_rl_bench',namespace='/v2')
        model=self.declare_parameter('model_path','/ws/output/v2/dynamics.xml').value
        policy=self.declare_parameter('policy_path','').value
        experimental=bool(self.declare_parameter('experimental',False).value)
        self.max_seconds=float(self.declare_parameter('max_seconds',30.).value)
        if not policy or not 0<self.max_seconds<=600:
            raise ValueError('Provide policy_path and a bounded max_seconds (0,600]')
        self.env=B2TransferEnv(model,randomize=False)
        self.actor=OnnxPolicy(policy,self.env.contract,experimental=experimental)
        response=self.declare_parameter('command_response','').value
        if response and not self.actor.metadata.get('reference_equivalent'):
            raise ValueError('Reference command calibration requires the unchanged upstream actor')
        self.adapter=VelocityAdapter(self.env.plant,response,self.env.contract['model_sha256']) if response else None
        self.estimate_status={}
        self.obs,_=self.env.reset(seed=30007,options={'command':[0.,0.,0.]})
        self.start_sim=float(self.env.plant.data.time)
        self.command=np.zeros(3);self.received=-math.inf
        self.fault='';self.finished=False;self.last_status=-math.inf
        self.create_subscription(Twist,'cmd_vel',self.on_command,10)
        self.odom=self.create_publisher(Odometry,'odom_truth',10)
        self.joints=self.create_publisher(JointState,'joint_states',10)
        self.imu=self.create_publisher(Imu,'imu_ideal',10)
        self.clock=self.create_publisher(Clock,'clock',10)
        self.status=self.create_publisher(String,'rl_status',10)
        self.timer=self.create_timer(.02,self.tick)
        self.get_logger().info('Experimental ONNX + free-base PD bench; all topics under /v2')

    def on_command(self,message):
        values=np.array([message.linear.x,message.linear.y,message.angular.z])
        if not np.isfinite(values).all():
            self.command[:]=0.;self.received=-math.inf
            return
        self.command=np.clip(values,[-.3,-.2,-.5],[.4,.2,.5])
        self.received=time.monotonic()

    def tick(self):
        if self.finished:return
        p=self.env.plant
        now=time.monotonic();stale=now-self.received>.25
        self.env.command=np.zeros(3) if stale or self.fault else self.command.copy()
        started=time.perf_counter()
        if not self.fault:
            try:
                self.obs=self.env.observation()
                if self.adapter:
                    self.obs[6:9],self.estimate_status=self.adapter.command(self.env.command,p)
                action,_=self.actor.predict(self.obs)
                self.obs,_,fallen,_,info=self.env.step(action)
                if fallen:self.fault='FALL_OR_TILT_LIMIT'
            except (ValueError,FloatingPointError,RuntimeError) as error:
                self.fault=f'POLICY_ERROR: {error}'
        if self.fault:
            # Latch: do not automatically resume a failed controller.
            p.kp[:]=0.;p.kd[:]=8.
            for _ in range(self.env.decimation):p.step(p.target)
        sim=float(p.data.time);s=p.state();t=stamp(sim)
        odom=Odometry();odom.header.stamp=t;odom.header.frame_id='v2_world';odom.child_frame_id='v2_base_link'
        xyz=p.data.xpos[p.base_id];q=p.data.xquat[p.base_id]
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
        odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z=map(float,q)
        odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=map(float,s['velocity'][:3])
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,s['velocity'][3:])
        self.odom.publish(odom)
        js=JointState();js.header.stamp=t;js.name=p.names
        js.position=p.data.qpos[p.qadr].tolist();js.velocity=p.data.qvel[p.dadr].tolist();js.effort=p.data.ctrl.tolist()
        self.joints.publish(js)
        imu=Imu();imu.header.stamp=t;imu.header.frame_id='v2_base_link'
        imu.orientation=odom.pose.pose.orientation;imu.angular_velocity=odom.twist.twist.angular
        # Acceleration is not fabricated from prescribed base commands.
        imu.linear_acceleration_covariance[0]=-1.
        self.imu.publish(imu);self.clock.publish(Clock(clock=t))
        if now-self.last_status>=.5:
            self.status.publish(String(data=json.dumps(dict(mode='experimental_free_base_onnx',runtime_ready=False,
                fault=self.fault,command_stale=stale,command=self.env.command.tolist(),height=s['height'],
                tilt_rad=float(np.arccos(np.clip(-s['gravity'][2],-1.,1.))),
                tick_ms=(time.perf_counter()-started)*1000.,simulation_seconds=sim-self.start_sim,
                velocity_adapter=bool(self.adapter),velocity_estimate=self.estimate_status))))
            self.last_status=now
        if sim-self.start_sim>=self.max_seconds:
            self.finished=True;self.timer.cancel()


def main():
    rclpy.init();node=None
    try:
        node=RlBench()
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if node:node.env.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
