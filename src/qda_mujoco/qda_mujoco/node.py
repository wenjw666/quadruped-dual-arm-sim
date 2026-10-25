"""ROS 2 adapter for the shared factory scene in MuJoCo.

RL mode uses free-base dynamics and bounded motor torques. The explicit
kinematic comparison mode prescribes root/joint poses, as in Gazebo V1.
The released cube uses MuJoCo contact dynamics. RGB-D and rays are rendered /
ray-cast from the actual MuJoCo scene, never replaced by target coordinates.
"""
import os
import json
os.environ.setdefault('MUJOCO_GL', 'egl')
import math
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import BPoly
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, LaserScan, Image, CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Float64MultiArray, Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory
from qda_interfaces.srv import Attach, Detach
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from ament_index_python.packages import get_package_share_directory
from qda_kinematics.kinematics import LEG_JOINTS, stance_command, AUBO_HOME_LEFT, AUBO_HOME_RIGHT
from qda_kinematics.scene import SPAWN_XYZ, EXPANDED


def stamp(seconds):
    ns = round(seconds*1e9)
    return Time(sec=ns//1000000000, nanosec=ns%1000000000)


class MujocoBackend(Node):
    def __init__(self):
        super().__init__('qda_mujoco')
        model_path = self.declare_parameter('model_path', '/ws/output/mujoco/factory.xml').value
        self.gui = bool(self.declare_parameter('gui', False).value)
        self.motion_mode=self.declare_parameter('motion_mode','kinematic').value
        self.controller=None
        self.allow_virtual_grasp=bool(self.declare_parameter('allow_virtual_grasp',False).value)
        if self.motion_mode=='rl':
            from .scene_control import SceneController
            self.controller=SceneController(model_path,
                self.declare_parameter('reference_model','/ws/output/v2/dynamics.xml').value,
                self.declare_parameter('policy_path','/ws/output/v2/runs/b2_transfer_v1/initial_policy.onnx').value,
                self.declare_parameter('command_response','/ws/output/v2_audit/b2_command_response.json').value)
            self.model,self.data=self.controller.plant.model,self.controller.plant.data
        elif self.motion_mode=='kinematic':
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        else:raise ValueError('motion_mode must be kinematic or rl')
        self.lock = threading.RLock()
        self.group = ReentrantCallbackGroup()
        self.running = True
        self.paused = False
        self.cmd = np.zeros(3)
        self.velocity = np.zeros(3)
        self.last_cmd = -1.0
        self.base = np.array([0.,0.,0.])
        self.targets = dict(zip(LEG_JOINTS,stance_command()))
        manifest=json.loads(Path(model_path).with_suffix('.manifest.json').read_text())
        self.mimics=manifest.get('mimics',{})
        for prefix,home in [('left',AUBO_HOME_LEFT),('right',AUBO_HOME_RIGHT)]:
            self.targets.update({f'{prefix}_joint_link{i+1}':v for i,v in enumerate(home)})
            self.targets.update({f'{prefix}_finger_{side}_joint':0. for side in ['left','right']})
        for name in self.mimics:self.targets[name]=0.
        self.joint_ids = {n:self.model.joint(n).id for n in self.targets}
        self.addresses = {n:int(self.model.jnt_qposadr[j]) for n,j in self.joint_ids.items()}
        self.dof_addresses = {n:int(self.model.jnt_dofadr[j]) for n,j in self.joint_ids.items()}
        self.root_adr = int(self.model.jnt_qposadr[self.model.joint('robot_root').id])
        self.object_adr = int(self.model.jnt_qposadr[self.model.joint('object_free').id])
        self.object_dof = int(self.model.jnt_dofadr[self.model.joint('object_free').id])
        self.plans = {}
        self.servo_lease = {}
        self.servo_received = {'left':0,'right':0}
        self.servo_velocity = {}
        self.servo_sim_stamp = {}
        self.servo_rejections = {'left':{},'right':{}}
        self.arm_modes_pub=self.create_publisher(String,'/motion/arm_modes',10)
        self.previous_joints = dict(self.targets)
        self.joint_velocity = dict.fromkeys(self.targets, 0.)
        self.held = None
        self.servers = []
        for prefix in ['left','right']:
            self.create_service(SetBool,f'/{prefix}_arm_controller/servo_mode',
                lambda q,r,s=prefix:self.servo_mode(s,q,r),callback_group=self.group)
            self.create_subscription(JointTrajectory,f'/{prefix}_arm_controller/servo_joint_trajectory',
                lambda msg,s=prefix:self.servo_target(s,msg),1,callback_group=self.group)
            for kind in ['arm','gripper']:
                name=f'{prefix}_{kind}_controller'
                names=[f'{prefix}_joint_link{i}' for i in range(1,8)] if kind=='arm' else [f'{prefix}_finger_{s}_joint' for s in ['left','right']]
                self.servers.append(ActionServer(self,FollowJointTrajectory,f'/{name}/follow_joint_trajectory',
                    execute_callback=lambda h,n=name,js=names:self.execute(h,n,js),
                    goal_callback=lambda g,js=names:self.validate_goal(g,js),
                    cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group))
        self.create_subscription(Twist,'/cmd_vel',self.on_cmd,10,callback_group=self.group)
        self.create_subscription(Float64MultiArray,'/leg_controller/commands',self.on_legs,10,callback_group=self.group)
        self.create_service(GetEntityState,'/get_entity_state',self.entity,callback_group=self.group)
        self.create_service(Attach,'/qda_grasp/attach',self.attach,callback_group=self.group)
        self.create_service(Detach,'/qda_grasp/detach',self.detach,callback_group=self.group)
        self.create_service(Empty,'/pause_physics',lambda q,r:self.set_pause(True,r),callback_group=self.group)
        self.create_service(Empty,'/unpause_physics',lambda q,r:self.set_pause(False,r),callback_group=self.group)
        self.clock_pub=self.create_publisher(Clock,'/clock',qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
        if self.controller:
            self.tf_pub=TransformBroadcaster(self)
            self.truth_pub=self.create_publisher(Odometry,'/simulation/odom_truth',10)
            self.rl_pub=self.create_publisher(String,'/locomotion/rl_status',10)
            self.ready_pub=self.create_publisher(Bool,'/locomotion/ready',10)
        self.imu_pub=self.create_publisher(Imu,'/imu',10)
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.cloud_pub=self.create_publisher(type(create_cloud_xyz32(Header(),[])),'/livox/lidar_PointCloud2',qos_profile_sensor_data)
        self.livox_pub=self.create_publisher(CustomMsg,'/livox/lidar',10)
        self.rgb_pub=self.create_publisher(Image,'/camera/image_raw',10)
        self.depth_pub=self.create_publisher(Image,'/camera/depth/image_raw',qos_profile_sensor_data)
        self.info_pub=self.create_publisher(CameraInfo,'/camera/camera_info',qos_profile_sensor_data)
        self.overview_pub=self.create_publisher(Image,'/overview/image_raw',qos_profile_sensor_data)
        self.aux_renderer=None;self.aux_publishers={}
        if self.controller:
            from .cameras import AUX_CAMERAS, optical_quaternion
            self.camera_tf=StaticTransformBroadcaster(self);transforms=[]
            for name,config in AUX_CAMERAS.items():
                frame=name+'_optical_frame'
                tf=TransformStamped(header=Header(frame_id=config['parent']),child_frame_id=frame)
                tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=config['xyz']
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,optical_quaternion(config))
                transforms.append(tf)
                self.aux_publishers[name]=(self.create_publisher(Image,f'/cameras/{name}/image_raw',qos_profile_sensor_data),
                    self.create_publisher(CameraInfo,f'/cameras/{name}/camera_info',qos_profile_sensor_data))
            self.camera_tf.sendTransform(transforms)
        csv=Path(get_package_share_directory('ros2_livox_simulation'))/'scan_mode/mid360.csv'
        angles=np.loadtxt(csv,delimiter=',',skiprows=1,usecols=(1,2))*math.pi/180
        az,zen=angles[:,0],angles[:,1]-math.pi/2
        self.pattern=np.column_stack((np.cos(az)*np.cos(zen),np.sin(az)*np.cos(zen),-np.sin(zen)))
        self.pattern_lines=np.clip(((zen+1.3)/.65).astype(int),0,3)
        self.pattern_index=0
        self.scan_angles=np.linspace(-3.14159,3.14159,360)
        self.scan_dirs=np.column_stack((np.cos(self.scan_angles),np.sin(self.scan_angles),np.zeros(360)))
        self.rng=np.random.default_rng(7)
        self.last_pub={}
        self.renderer=None
        self.render_option=mujoco.MjvOption()
        self.render_option.geomgroup[3]=1
        self.viewer=None
        if not self.controller:self.apply_robot()
        mujoco.mj_forward(self.model,self.data)
        self.get_logger().info(f'MuJoCo {mujoco.__version__}: {len(self.targets)} joints, motion={self.motion_mode}, shared scene')

    def on_cmd(self,msg):
        with self.lock:
            values=np.array([msg.linear.x,msg.linear.y,msg.angular.z])
            limits=([-.05,-.08,-.25],[.15,.08,.25]) if self.controller else ([-.05,-.12,-.7],[.22,.12,.7])
            self.cmd=np.clip(values,*limits) if np.all(np.isfinite(values)) else np.zeros(3)
            self.last_cmd=self.data.time

    def on_legs(self,msg):
        if self.controller:return # RL exclusively owns the leg actuators.
        if len(msg.data)==12 and np.all(np.isfinite(msg.data)):
            with self.lock:self.targets.update(zip(LEG_JOINTS,msg.data))

    def set_pause(self,paused,response):
        self.paused=paused
        return response

    def validate_goal(self,goal,names):
        with self.lock:
            self.expire_servo()
            if any(n.startswith(side+'_joint_link') for side in self.servo_lease for n in names):return GoalResponse.REJECT
        if self.controller and (not self.controller.ready or self.controller.fault):return GoalResponse.REJECT
        jt=goal.trajectory
        if set(jt.joint_names)!=set(names) or not jt.points:return GoalResponse.REJECT
        previous=-1.
        for point in jt.points:
            t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
            if t<=previous or len(point.positions)!=len(names) or not np.all(np.isfinite(point.positions)):return GoalResponse.REJECT
            if self.controller:
                for name,value in zip(jt.joint_names,point.positions):
                    lo,hi=self.model.jnt_range[self.joint_ids[name]]
                    if not lo<=value<=hi:return GoalResponse.REJECT
            for field in (point.velocities,point.accelerations):
                if field and (len(field)!=len(names) or not np.all(np.isfinite(field))):return GoalResponse.REJECT
            previous=t
        return GoalResponse.ACCEPT

    def expire_servo(self):
        for side,until in list(self.servo_lease.items()):
            if time.monotonic()>until:
                self.servo_lease.pop(side,None)
                self.servo_velocity.pop(side,None)
                for i in range(1,8):
                    name=f'{side}_joint_link{i}';self.targets[name]=float(self.data.qpos[self.addresses[name]])
                    if self.controller:self.controller.plant.velocity_reference[self.controller.plant.names.index(name)]=0.

    def servo_mode(self,side,request,response):
        with self.lock:
            self.expire_servo()
            if request.data:
                if (not self.controller or not self.controller.ready or self.controller.fault
                        or f'{side}_arm_controller' in self.plans or side in self.servo_lease):
                    response.success=False;response.message='Controller unavailable or already owned';return response
                self.servo_lease[side]=time.monotonic()+5.
                # Incremental position commands otherwise erase the static PD
                # position error needed to balance gravity, making the arm sag.
                # Keep bounded motor feedforward active during subsequent holds.
                plant=self.controller.plant
                for i in range(1,8):
                    joint=f'{side}_joint_link{i}'
                    plant.bias_compensation_mask[plant.names.index(joint)]=True
                    self.targets[joint]=float(self.data.qpos[self.addresses[joint]])
            else:
                self.servo_lease.pop(side,None)
                self.servo_velocity.pop(side,None)
                for i in range(1,8):
                    name=f'{side}_joint_link{i}';self.targets[name]=float(self.data.qpos[self.addresses[name]])
                    if self.controller:self.controller.plant.velocity_reference[self.controller.plant.names.index(name)]=0.
            response.success=True;response.message='Servo owns arm' if request.data else 'Position held'
            return response

    def servo_target(self,side,msg):
        def reject(reason):
            counts=self.servo_rejections[side];counts[reason]=counts.get(reason,0)+1
        with self.lock:
            self.expire_servo()
            if side not in self.servo_lease or f'{side}_arm_controller' in self.plans:return reject('unowned')
            if not self.controller or self.controller.fault:return reject('fault')
            names=[f'{side}_joint_link{i}' for i in range(1,8)]
            if len(msg.joint_names)!=7 or set(msg.joint_names)!=set(names) or not msg.points:return reject('joints')
            point=msg.points[0]
            if len(point.positions)!=7 or not np.all(np.isfinite(point.positions)):return reject('nonfinite')
            if len(point.velocities)!=7 or not np.all(np.isfinite(point.velocities)) or max(abs(v) for v in point.velocities)>1.5:return reject('velocity')
            sent=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            # Humble MoveIt Servo intentionally emits stamp=0 (execute now).
            # For that convention the exclusive lease, depth-1 queue and receipt
            # watchdog apply; timestamped trajectories also have an age gate.
            age=self.data.time-sent
            if sent and (age>.4 or age<-.1):return reject('stale')
            for name,value in zip(msg.joint_names,point.positions):
                lo,hi=self.model.jnt_range[self.joint_ids[name]]
                if not lo<=value<=hi:return reject('joint_limit')
                if abs(value-self.data.qpos[self.addresses[name]])>.08:return reject('joint_step')
            # Servo runs on wall timers. Integrate its bounded velocity output
            # using physics time, not the number of wall-clock messages.
            self.servo_velocity[side]=dict(zip(msg.joint_names,point.velocities))
            self.servo_sim_stamp[side]=self.data.time
            self.servo_received[side]+=1
            self.servo_lease[side]=time.monotonic()+5.

    def execute(self,handle,name,names):
        jt=handle.request.trajectory
        order=[jt.joint_names.index(n) for n in names]
        with self.lock:
            self.expire_servo()
            if name in self.plans or (name.endswith('_arm_controller') and name.split('_')[0] in self.servo_lease):
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-1,error_string='Controller already executing')
            times=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points]
            points=np.array([[p.positions[i] for i in order] for p in jt.points])
            derivatives = [[np.array([p.positions[i] for i in order])] for p in jt.points]
            if all(len(p.velocities)==len(names) for p in jt.points):
                for row,p in zip(derivatives,jt.points):row.append(np.array([p.velocities[i] for i in order]))
                if all(len(p.accelerations)==len(names) for p in jt.points):
                    for row,p in zip(derivatives,jt.points):row.append(np.array([p.accelerations[i] for i in order]))
            if times[0]>0:
                times.insert(0,0.)
                points=np.vstack(([self.targets[n] for n in names],points))
                derivatives.insert(0,[points[0]]+[np.zeros(len(names)) for _ in range(len(derivatives[0])-1)])
            spline = BPoly.from_derivatives(times,derivatives) if len(times)>1 else None
            plan=dict(start=self.data.time,times=np.array(times),points=points,names=names,done=False,spline=spline)
            self.plans[name]=plan
        while self.running and rclpy.ok():
            if self.controller and self.controller.fault:
                with self.lock:self.plans.pop(name,None)
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-4,error_string=self.controller.fault)
            if handle.is_cancel_requested:
                with self.lock:self.plans.pop(name,None)
                handle.canceled()
                return FollowJointTrajectory.Result(error_code=0,error_string='Cancelled; position held')
            with self.lock:
                done=plan['done']
                error=max(abs(self.data.qpos[self.addresses[n]]-self.targets[n]) for n in names)
                expired=self.data.time-plan['start']>plan['times'][-1]+3.
            if self.controller and done and error>.06:
                if expired:
                    with self.lock:self.plans.pop(name,None)
                    handle.abort()
                    return FollowJointTrajectory.Result(error_code=-5,error_string=f'Physical tracking error {error:.3f} rad')
                time.sleep(.01);continue
            if done:
                with self.lock:self.plans.pop(name,None)
                handle.succeed()
                return FollowJointTrajectory.Result(error_code=0)
            time.sleep(.01)
        handle.abort()
        return FollowJointTrajectory.Result(error_code=-1,error_string='Backend stopped')

    def apply_robot(self):
        for name,mimic in self.mimics.items():
            self.targets[name]=self.targets[mimic['joint']]*float(mimic.get('multiplier',1))+float(mimic.get('offset',0))
        a=self.root_adr
        self.data.qpos[a:a+3]=[self.base[0],self.base[1],SPAWN_XYZ[2]]
        self.data.qpos[a+3:a+7]=[math.cos(self.base[2]/2),0,0,math.sin(self.base[2]/2)]
        self.data.qvel[:6]=0
        for name,value in self.targets.items():
            self.data.qpos[self.addresses[name]]=value
            self.data.qvel[self.dof_addresses[name]]=0

    def entity(self,request,response):
        name='base_link' if request.name=='qda_robot' else request.name
        try:
            with self.lock:
                bid=self.model.body(name).id
                xyz=self.data.xpos[bid].copy(); q=self.data.xquat[bid].copy()
            if request.reference_frame not in ('','world','odom'):
                response.status_message='Only world/odom reference supported';return response
            response.state.name=request.name
            response.state.pose.position.x,response.state.pose.position.y,response.state.pose.position.z=map(float,xyz)
            response.state.pose.orientation.w,response.state.pose.orientation.x,response.state.pose.orientation.y,response.state.pose.orientation.z=map(float,q)
            response.success=True
        except KeyError:response.status_message='Unknown entity'
        return response

    def attach(self,request,response):
        if self.controller and not self.allow_virtual_grasp:
            response.message='Virtual grasp disabled in RL mode; physical grasp verification pending'
            return response
        if request.parent_model!='qda_robot' or request.child_model!='pick_object' or request.child_link!='object_link':
            response.message='Unsupported attachment';return response
        try:
            with self.lock:
                bid=self.model.body(request.parent_link).id
                obj=self.model.body('pick_object').id
                rot=self.data.xmat[bid].reshape(3,3)
                delta=rot.T@(self.data.xpos[obj]-self.data.xpos[bid])
                inv=self.data.xquat[bid].copy();inv[1:]*=-1
                relative=np.zeros(4);mujoco.mju_mulQuat(relative,inv,self.data.xquat[obj])
                self.held=(bid,delta,relative)
            response.success=True;response.message='MuJoCo attached pick_object (virtual fixed grasp)'
        except KeyError:response.message='Unknown parent link'
        return response

    def detach(self,request,response):
        if request.child_model!='pick_object':response.message='Unknown object';return response
        with self.lock:
            self.held=None
            self.data.qvel[self.object_dof:self.object_dof+6]=0
        response.success=True;response.message='MuJoCo released pick_object'
        return response

    def update_held(self):
        if self.held is not None:
            bid,delta,relative=self.held
            a=self.object_adr
            self.data.qpos[a:a+3]=self.data.xpos[bid]+self.data.xmat[bid].reshape(3,3)@delta
            mujoco.mju_mulQuat(self.data.qpos[a+3:a+7],self.data.xquat[bid],relative)
            self.data.qvel[self.object_dof:self.object_dof+6]=0

    def due(self,key,period):
        if self.data.time-self.last_pub.get(key,-1)>period-1e-8:
            self.last_pub[key]=self.data.time
            return True
        return False

    def header(self,frame):return Header(stamp=stamp(self.data.time),frame_id=frame)

    def rays(self,frame,directions,maximum):
        bid=self.model.body(frame).id
        world_dirs=np.ascontiguousarray(directions@self.data.xmat[bid].reshape(3,3).T).reshape(-1)
        origin=self.data.xpos[bid].copy()
        geomids=np.full(len(directions),-1,dtype=np.int32)
        distances=np.full(len(directions),-1.,dtype=np.float64)
        # mj_ray ignores transparent geoms. World visual primitives are exact
        # duplicates of their collision shapes; group 3 excludes robot visuals.
        mujoco.mj_multiRay(self.model,self.data,origin,world_dirs,np.array([1,0,0,1,0,0],dtype=np.uint8),1,-1,geomids,distances,len(directions),maximum)
        return distances

    def publish_state(self,acceleration):
        if self.controller:
            self.publish_dynamic_state()
        else:
            self.publish_kinematic_state(acceleration)
        self.publish_lidar()

    def publish_kinematic_state(self,acceleration):
        if self.due('clock',.01):self.clock_pub.publish(Clock(clock=stamp(self.data.time)))
        if self.due('state',.02):
            odom=Odometry(header=self.header('odom'),child_frame_id='base_link')
            odom.pose.pose.position.x,odom.pose.pose.position.y=map(float,self.base[:2]);odom.pose.pose.position.z=SPAWN_XYZ[2]
            odom.pose.pose.orientation.z=math.sin(self.base[2]/2);odom.pose.pose.orientation.w=math.cos(self.base[2]/2)
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=map(float,self.velocity)
            self.odom_pub.publish(odom)
            js=JointState(header=self.header('base_link'),name=list(self.targets),position=[float(self.data.qpos[self.addresses[n]]) for n in self.targets],velocity=[float(self.joint_velocity[n]) for n in self.targets])
            self.joint_pub.publish(js)
        if self.due('imu',.005):
            imu=Imu(header=self.header('imu_link'))
            imu.orientation.w=math.cos(self.base[2]/2);imu.orientation.z=math.sin(self.base[2]/2)
            imu.angular_velocity.z=float(self.velocity[2])
            imu.linear_acceleration.x=float(acceleration[0]-self.velocity[2]*self.velocity[1])
            imu.linear_acceleration.y=float(acceleration[1]+self.velocity[2]*self.velocity[0])
            imu.linear_acceleration.z=9.81
            self.imu_pub.publish(imu)

    def publish_dynamic_state(self):
        p=self.controller.plant;s=p.state()
        if self.due('clock',.01):self.clock_pub.publish(Clock(clock=stamp(self.data.time)))
        if self.due('state',.02):
            odom=Odometry(header=self.header('odom'),child_frame_id='base_link')
            xyz=self.data.xpos[p.base_id];q=self.data.xquat[p.base_id]
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
            odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z=map(float,q)
            odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=map(float,s['velocity'][:3])
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,s['velocity'][3:])
            self.odom_pub.publish(odom);self.truth_pub.publish(odom)
            transform=TransformStamped(header=odom.header,child_frame_id='base_link')
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,xyz)
            transform.transform.rotation=odom.pose.pose.orientation;self.tf_pub.sendTransform(transform)
            self.joint_pub.publish(JointState(header=self.header('base_link'),name=list(self.targets),
                position=[float(self.data.qpos[self.addresses[n]]) for n in self.targets],
                velocity=[float(self.data.qvel[self.dof_addresses[n]]) for n in self.targets]))
        if self.due('imu',.005):
            imu=Imu(header=self.header('imu_link'))
            q=self.data.xquat[p.base_id]
            imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z=map(float,q)
            imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,s['velocity'][:3])
            # Full accelerometer simulation is a later sensor-stage requirement.
            imu.linear_acceleration_covariance[0]=-1.
            self.imu_pub.publish(imu)
        if self.due('rl_status',.2):
            self.arm_modes_pub.publish(String(data=json.dumps({'servo':list(self.servo_lease),
                'trajectory':list(self.plans),'accepted_servo_messages':self.servo_received,
                'rejected_servo_messages':self.servo_rejections,
                'bias_compensated_joints':[n for n,on in zip(self.controller.plant.names,self.controller.plant.bias_compensation_mask) if on]})))
            status=self.controller.status();status['grasp']='virtual_fixed_attachment' if self.allow_virtual_grasp else 'disabled_pending_physical_verification'
            self.rl_pub.publish(String(data=json.dumps(status)))
            self.ready_pub.publish(Bool(data=self.controller.ready))

    def publish_lidar(self):
        if self.due('scan',.1):
            dist=self.rays('nav_scan_link',self.scan_dirs,12)
            dist[(dist<.35)|(dist>12)]=np.inf
            scan=LaserScan(header=self.header('nav_scan_link'),angle_min=-3.14159,angle_max=3.14159,angle_increment=6.28318/359,scan_time=.1,range_min=.35,range_max=12.)
            scan.ranges=dist.astype(np.float32).tolist();self.scan_pub.publish(scan)
            indices=(np.arange(2400)+self.pattern_index)%len(self.pattern)
            self.pattern_index=(self.pattern_index+2400)%len(self.pattern)
            rays=self.pattern[indices]
            dist=self.rays('lidar_link',rays,20.)
            valid=(dist>.15)&(dist<20.)
            xyz=rays*np.where(valid,dist,0)[:,None]
            header=self.header('lidar_link')
            self.cloud_pub.publish(create_cloud_xyz32(header,xyz[valid].astype(np.float32)))
            # Rays into open sky have no return; do not encode them as origin
            # points or send an all-zero packet into LIVO's point preprocessing.
            returned=np.flatnonzero(valid)
            if len(returned):
                livox=CustomMsg(header=header,timebase=round(self.data.time*1e9),point_num=len(returned))
                livox.points=[CustomPoint(x=float(xyz[i,0]),y=float(xyz[i,1]),z=float(xyz[i,2]),reflectivity=80,tag=0,line=int(self.pattern_lines[indices[i]]),offset_time=int(i/2400*1e8)) for i in returned]
                self.livox_pub.publish(livox)

    def image(self,array,encoding,frame):
        a=np.ascontiguousarray(array)
        return Image(header=self.header(frame),height=a.shape[0],width=a.shape[1],encoding=encoding,is_bigendian=0,step=a.strides[0],data=a.tobytes())

    def render_sensors(self):
        if self.due('camera',.1):
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data,camera='head_camera',scene_option=self.render_option)
            rgb=self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.data,camera='head_camera',scene_option=self.render_option)
            depth=self.renderer.render().copy().astype(np.float32)
            depth[(depth<.05)|(depth>8.)]=np.nan
            self.rgb_pub.publish(self.image(rgb,'rgb8','camera_optical_frame'))
            self.depth_pub.publish(self.image(depth,'32FC1','camera_optical_frame'))
            fovy=self.model.cam_fovy[self.model.camera('head_camera').id]*math.pi/180
            focal=240/math.tan(fovy/2)
            info=CameraInfo(header=self.header('camera_optical_frame'),height=480,width=640,distortion_model='plumb_bob',d=[0.]*5)
            info.k=[focal,0.,320.,0.,focal,240.,0.,0.,1.]
            info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            info.p=[focal,0.,320.,0.,0.,focal,240.,0.,0.,0.,1.,0.]
            self.info_pub.publish(info)
        if self.aux_renderer and self.due('aux_cameras',.2):
            for name,(rgb_pub,info_pub) in self.aux_publishers.items():
                self.aux_renderer.update_scene(self.data,camera=name,scene_option=self.render_option)
                frame=name+'_optical_frame'
                rgb_pub.publish(self.image(self.aux_renderer.render(),'rgb8',frame))
                focal=120/math.tan(math.radians(self.model.cam_fovy[self.model.camera(name).id])/2)
                info=CameraInfo(header=self.header(frame),height=240,width=320,distortion_model='plumb_bob',d=[0.]*5)
                info.k=[focal,0.,160.,0.,focal,120.,0.,0.,1.]
                info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                info.p=[focal,0.,160.,0.,0.,focal,120.,0.,0.,0.,1.,0.]
                info_pub.publish(info)
        if self.due('overview',.25):
            self.renderer.disable_depth_rendering();self.renderer.update_scene(self.data,camera='overview',scene_option=self.render_option)
            self.overview_pub.publish(self.image(self.renderer.render(),'rgb8','overview_cam'))

    def run(self):
        self.renderer=mujoco.Renderer(self.model,height=480,width=640)
        if self.aux_publishers:self.aux_renderer=mujoco.Renderer(self.model,height=240,width=320)
        if self.gui:
            from mujoco import viewer
            self.viewer=viewer.launch_passive(self.model,self.data)
            self.viewer.opt.geomgroup[3]=1
            self.viewer.cam.lookat[:]=[1.1,-.25,.6];self.viewer.cam.distance=5.5
            if EXPANDED:self.viewer.cam.lookat[:]=[2.5,.35,.5];self.viewer.cam.distance=10.5
            self.viewer.cam.azimuth=120;self.viewer.cam.elevation=-35
        dt=self.model.opt.timestep
        try:
            while self.running and rclpy.ok():
                wall=time.monotonic()
                if self.paused:
                    time.sleep(.01);continue
                if self.controller:
                    with self.lock:
                        self.step_dynamic()
                        self.publish_state(np.zeros(3));self.render_sensors()
                        if self.viewer and self.due('viewer',1/20):self.viewer.sync()
                    # Yield outside the data lock so action/service callbacks can
                    # acquire control even while software rendering runs slowly.
                    time.sleep(max(0.,dt-(time.monotonic()-wall)))
                    continue
                with self.lock:
                    target=self.cmd if self.data.time-self.last_cmd<=.5 else np.zeros(3)
                    previous=self.velocity.copy()
                    self.velocity+=np.clip(target-self.velocity,-np.array([.35,.25,.8])*dt,np.array([.35,.25,.8])*dt)
                    acceleration=(self.velocity-previous)/dt
                    vx,vy,wz=self.velocity
                    angle=self.base[2]+.5*wz*dt
                    self.base+=np.array([math.cos(angle)*vx-math.sin(angle)*vy,math.sin(angle)*vx+math.cos(angle)*vy,wz])*dt
                    for plan in list(self.plans.values()):
                        elapsed=self.data.time-plan['start']
                        t=np.clip(elapsed,plan['times'][0],plan['times'][-1])
                        values=plan['spline'](t) if plan['spline'] is not None else plan['points'][-1]
                        for i,n in enumerate(plan['names']):self.targets[n]=float(values[i])
                        plan['done']=elapsed>=plan['times'][-1]
                    self.joint_velocity={n:(v-self.previous_joints[n])/dt for n,v in self.targets.items()}
                    self.previous_joints=dict(self.targets)
                    self.apply_robot();mujoco.mj_forward(self.model,self.data);self.update_held()
                    mujoco.mj_step(self.model,self.data)
                    self.apply_robot();mujoco.mj_forward(self.model,self.data);self.update_held();mujoco.mj_forward(self.model,self.data)
                    self.publish_state(acceleration)
                    self.render_sensors()
                    if self.viewer and self.due('viewer',1/20):self.viewer.sync()
                delay=dt-(time.monotonic()-wall)
                if delay>0:time.sleep(delay)
        finally:
            if self.viewer:self.viewer.close()
            if self.renderer:self.renderer.close()
            if self.aux_renderer:self.aux_renderer.close()

    def step_dynamic(self):
        self.expire_servo()
        for side in self.servo_lease:
            fresh=self.data.time-self.servo_sim_stamp.get(side,-1)<.35
            for i in range(1,8):
                name=f'{side}_joint_link{i}';velocity=self.servo_velocity.get(side,{}).get(name,0.) if fresh else 0.
                current=float(self.data.qpos[self.addresses[name]])
                lo,hi=self.model.jnt_range[self.joint_ids[name]]
                target=self.targets[name]+velocity*self.model.opt.timestep
                self.targets[name]=float(np.clip(target,max(lo,current-.08),min(hi,current+.08)))
                self.controller.plant.velocity_reference[self.controller.plant.names.index(name)]=velocity
        for plan in list(self.plans.values()):
            elapsed=self.data.time-plan['start'];t=np.clip(elapsed,plan['times'][0],plan['times'][-1])
            values=plan['spline'](t) if plan['spline'] is not None else plan['points'][-1]
            for i,name in enumerate(plan['names']):self.targets[name]=float(values[i])
            plan['done']=elapsed>=plan['times'][-1]
        target=self.cmd if self.data.time-self.last_cmd<=.25 else np.zeros(3)
        self.controller.step(target,self.targets)
        mujoco.mj_forward(self.model,self.data)
        if self.held is not None:self.update_held();mujoco.mj_forward(self.model,self.data)


def main():
    rclpy.init()
    node=MujocoBackend()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    try:node.run()
    except KeyboardInterrupt:pass
    finally:
        node.running=False
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
