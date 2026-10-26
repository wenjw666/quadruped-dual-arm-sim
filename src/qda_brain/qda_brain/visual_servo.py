"""Marker IBVS -> MoveIt Servo -> exclusive bounded joint targets."""
import json
import threading
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String,Int8
from std_srvs.srv import SetBool,Trigger
from cv_bridge import CvBridge
from tf2_ros import Buffer,TransformListener
from qda_interfaces.action import VisualAlign
from qda_interfaces.srv import BrainRequest
from .ibvs import command
from .feature_tracker import MarkerTracker


class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servo');self.group=ReentrantCallbackGroup();self.lock=threading.Lock()
        self.bridge=CvBridge();self.frames={};self.infos={};self.states={};self.base={};self.base_at=0.
        self.arm_modes={};self.modes_at=0.
        # This offline renderer runs below real time. Simulated image age remains
        # bounded separately; plant/clock watchdogs still stop after 5 wall seconds.
        self.image_wall_timeout=self.declare_parameter('image_wall_timeout',15.).value
        self.trackers={};self.feature_source={};self.klt_frames={}
        self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.extrinsics={}
        self.twist_pubs={};self.modes={};self.starts={};self.stops={};self.leases={}
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.metrics=self.create_publisher(String,'/brain/servo_metrics',10)
        self.overlay=self.create_publisher(Image,'/brain/servo_overlay',10)
        self.create_subscription(String,'/locomotion/rl_status',self.base_state,10,callback_group=self.group)
        self.create_subscription(String,'/motion/arm_modes',self.arm_state,10,callback_group=self.group)
        for side in ('left','right'):
            self.create_subscription(Image,f'/cameras/{side}_wrist/image_raw',lambda m,s=side:self.frame(s,m),qos_profile_sensor_data,callback_group=self.group)
            self.create_subscription(CameraInfo,f'/cameras/{side}_wrist/camera_info',lambda m,s=side:self.infos.update({s:m}),qos_profile_sensor_data,callback_group=self.group)
            self.create_subscription(Int8,f'/servo_{side}/status',lambda m,s=side:self.states.update({s:m.data}),10,callback_group=self.group)
            self.twist_pubs[side]=self.create_publisher(TwistStamped,f'/servo_{side}/delta_twist_cmds',10)
            self.modes[side]=self.create_client(SetBool,f'/{side}_arm_controller/servo_mode',callback_group=self.group)
            self.leases[side]=self.create_client(BrainRequest,f'/{side}_arm_controller/servo_lease',callback_group=self.group)
            self.starts[side]=self.create_client(Trigger,f'/servo_{side}/start_servo',callback_group=self.group)
            self.stops[side]=self.create_client(Trigger,f'/servo_{side}/stop_servo',callback_group=self.group)
        self.server=ActionServer(self,VisualAlign,'/visual_align',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)

    def base_state(self,msg):self.base=json.loads(msg.data);self.base_at=time.monotonic()
    def arm_state(self,msg):self.arm_modes=json.loads(msg.data);self.modes_at=time.monotonic()
    def frame(self,side,msg):self.frames[side]=(msg,time.monotonic())

    def accept(self,g):
        if g.arm not in ('left','right') or not 0<=g.marker_id<50 or not .15<=g.desired_depth<=.8 or not 1<=g.timeout_seconds<=180:return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self.lock.acquire(False) else GoalResponse.REJECT

    def call(self,client,request):
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('SERVO_SERVICE_UNAVAILABLE')
        f=client.call_async(request);deadline=time.monotonic()+3
        while not f.done():
            if time.monotonic()>deadline:raise TimeoutError('SERVO_SERVICE_TIMEOUT:'+getattr(client,'srv_name','unknown'))
            time.sleep(.01)
        r=f.result()
        if hasattr(r,'response_json'):
            result=json.loads(r.response_json)
            if not result.get('ok'):raise RuntimeError(result.get('error','SERVO_LEASE_REJECTED'))
        elif not r.success:raise RuntimeError(r.message)
        return r

    def twist(self,side,velocity=None):
        msg=TwistStamped();msg.header.frame_id=side+'_gripper';msg.header.stamp=self.get_clock().now().to_msg()
        if velocity is not None:
            # Servo's Jacobian/IK tip is the SRDF gripper, not the camera mount.
            # Transform both axes AND the velocity reference point using TF.
            rotation,translation=self.extrinsics[side]
            omega=rotation@velocity[3:];linear=rotation@velocity[:3]+np.cross(translation,omega)
            msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z=map(float,linear)
            msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z=map(float,omega)
        self.twist_pubs[side].publish(msg)

    def features(self,side,marker):
        if side not in self.frames or side not in self.infos:raise ValueError('NO_CAMERA')
        msg,received=self.frames[side]
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if time.monotonic()-received>self.image_wall_timeout or not -.05<=age<=.5:
            raise ValueError(f'STALE_IMAGE(sim_age={age:.3f},wall_age={time.monotonic()-received:.3f})')
        image=self.bridge.imgmsg_to_cv2(msg,'bgr8').copy();gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        key=(side,marker)
        if key not in self.trackers:self.trackers[key]=MarkerTracker(marker)
        pts,source,misses=self.trackers[key].update(gray,stamp)
        self.feature_source[side]=source;self.klt_frames[side]=misses
        info=self.infos[side];k=np.asarray(info.k).reshape(3,3)
        rvec,tvec,_=cv2.aruco.estimatePoseSingleMarkers(pts.reshape(1,4,2),.08,k,np.zeros(5))
        rot,_=cv2.Rodrigues(rvec[0]);points=np.array([[-.04,.04,0],[.04,.04,0],[.04,-.04,0],[-.04,-.04,0]])
        projected,_=cv2.projectPoints(points,rvec[0],tvec[0],k,np.zeros(5))
        if not np.all(np.isfinite(tvec)) or np.sqrt(np.mean((projected.reshape(4,2)-pts)**2))>2.:
            raise ValueError('PNP_REPROJECTION_REJECTED')
        depths=(points@rot.T+tvec[0].reshape(3))[:,2]
        cv2.aruco.drawDetectedMarkers(image,[pts.reshape(1,4,2)],np.array([[marker]]))
        overlay=self.bridge.cv2_to_imgmsg(image,'bgr8');overlay.header=msg.header;self.overlay.publish(overlay)
        return pts,depths,k,(msg.header.stamp.sec,msg.header.stamp.nanosec)

    def execute(self,handle):
        g=handle.request;side=g.arm;initial=0.;error=0.;owned=False;started=False;success=False;code=''
        try:
            self.trackers.pop((side,g.marker_id),None)
            self.features(side,g.marker_id) # Do not take actuator ownership without an observation.
            transform=self.tf.lookup_transform(side+'_gripper',side+'_wrist_optical_frame',rclpy.time.Time()).transform
            q=transform.rotation;t=transform.translation
            self.extrinsics[side]=(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.array([t.x,t.y,t.z]))
            response=self.call(self.modes[side],SetBool.Request(data=True));owned=True
            token=json.loads(response.message)['token'];renewed=time.monotonic()
            self.call(self.starts[side],Trigger.Request());started=True
            self.states[side]=0
            acquired=time.monotonic()
            deadline=time.monotonic()+g.timeout_seconds;last=None;stable=0;previous=np.zeros(6);lost_at=None
            while time.monotonic()<deadline:
                if handle.is_cancel_requested:raise RuntimeError('CANCELLED')
                if time.monotonic()-renewed>1.:
                    self.call(self.leases[side],BrainRequest.Request(request_json=json.dumps({'op':'renew','token':token})))
                    renewed=time.monotonic()
                if time.monotonic()-acquired>5 and (side not in self.arm_modes.get('servo',[]) or time.monotonic()-self.modes_at>5):raise RuntimeError('SERVO_OWNERSHIP_LOST')
                v=self.base.get('actual_body_velocity',[1,1,1])
                if self.base.get('fault') or time.monotonic()-self.base_at>5 or np.linalg.norm(v[:2])>.04 or self.base.get('tilt_rad',1)>.1:
                    raise RuntimeError(f'BASE_UNSTABLE(age={time.monotonic()-self.base_at:.2f},v={np.linalg.norm(v[:2]):.3f},tilt={self.base.get("tilt_rad",1):.3f})')
                if self.states.get(side,0) in (2,4,5):raise RuntimeError('MOVEIT_SERVO_HALT_'+str(self.states[side]))
                cached=self.frames.get(side)
                if cached and (cached[0].header.stamp.sec,cached[0].header.stamp.nanosec)==last:
                    age=(self.get_clock().now().nanoseconds-(last[0]*10**9+last[1]))*1e-9
                    if time.monotonic()-cached[1]>self.image_wall_timeout or age>.5:
                        self.twist(side);raise RuntimeError('STALE_IMAGE_WHILE_WAITING')
                    time.sleep(.02);continue
                try:pts,z,k,stamp=self.features(side,g.marker_id)
                except ValueError as exc:
                    self.twist(side);previous[:]=0;stable=0
                    if lost_at is None:lost_at=time.monotonic()
                    if time.monotonic()-lost_at>1:raise RuntimeError(str(exc))
                    time.sleep(.04);continue
                lost_at=None
                if stamp==last:time.sleep(.02);continue
                last=stamp
                velocity,error,condition=command(pts,z,k,g.desired_depth)
                # Per-frame acceleration bound for the nominal 5 Hz wrist stream.
                velocity=np.clip(velocity,previous-np.array([.008]*3+[.04]*3),previous+np.array([.008]*3+[.04]*3));previous=velocity
                if not initial:initial=error
                stable=stable+1 if error<1.5 and self.feature_source.get(side)=='aruco' else 0
                self.metrics.publish(String(data=json.dumps(dict(arm=side,error_px=error,condition=condition,depth_m=float(np.mean(z)),source='aruco_pnp_ibvs',
                    servo_status=self.states.get(side),arm_modes=self.arm_modes,velocity=velocity.tolist(),
                    feature_source=self.feature_source.get(side),klt_bridge_frames=self.klt_frames.get(side,0)))))
                handle.publish_feedback(VisualAlign.Feedback(error_px=error,state='tracking'))
                if stable>=4:success=True;break
                self.twist(side,velocity);time.sleep(.02)
            if not success:raise TimeoutError('ALIGN_TIMEOUT')
            handle.succeed()
        except Exception as exc:
            code=str(exc)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
        finally:
            self.twist(side)
            for active,client,request in ((started,self.stops[side],Trigger.Request()),(owned,self.modes[side],SetBool.Request(data=False))):
                if active:
                    try:self.call(client,request)
                    except Exception as exc:self.get_logger().error(str(exc))
            self.lock.release()
        return VisualAlign.Result(success=success,error_code=code,initial_error_px=initial,final_error_px=error)


def main():
    rclpy.init();node=VisualServo();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
