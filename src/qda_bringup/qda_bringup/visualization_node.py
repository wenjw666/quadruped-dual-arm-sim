"""Bounded, backend-neutral diagnostics for RViz; all curves use measured data."""
from collections import deque
import math
import json
import time
from pathlib import Path as FilePath

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import DisplayTrajectory
from tf2_ros import Buffer, TransformListener
from qda_kinematics.kinematics import _aubo_fk, _LEFT_MOUNT
from qda_kinematics.assembly import LEFT_ARM_BASE
from qda_kinematics.scene import PICK_TABLE_XYZ, PLACE_TABLE_XYZ, TABLE_SIZE, RACK_XYZ, RACK_SIZE
from qda_kinematics.scene import SCENARIO, EXPANDED, ROUTE_OPTIONS, ROUTE_WAYPOINTS, planning_boxes
from qda_kinematics.scene import PLACE_YAW, PLACE_MARKER_XYZ, APPROACH_IN_BASE


class AlgorithmView(Node):
    def __init__(self):
        super().__init__('algorithm_view')
        self.backend = self.declare_parameter('backend', 'gazebo').value
        self.output_dir = FilePath(self.declare_parameter('output_dir', '/ws/output').value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pose = None
        self.stage = 'Waiting for task'
        if not self.declare_parameter('task_enabled',True).value:self.stage='Manual mode - no task launched'
        self.last_task_wall = 0.0
        self.labels = 'No detection yet'
        self.cmd = Twist()
        self.base_points = deque(maxlen=2000)
        self.tool_points = deque(maxlen=2000)
        self.sim_previous = None
        self.wall_previous = time.monotonic()
        self.rtf = 0.0
        self.livo_last = None
        self.livo_wall = None
        self.plan = None
        self.joints = {}
        self.metrics = {}
        self.rl_status={};self.command_owner='waiting'
        self.brain_status={};self.servo_metrics={};self.servo_wall=0.
        self.mission_status={};self.module_health={}
        self.manipulation={};self.manipulation_wall=0.;self.object_points=deque(maxlen=2000)
        self.create_subscription(String,'/manipulation/status',self.on_manipulation,10)
        self.create_subscription(String,'/brain/mission_status',lambda m:self.on_json(m,'mission_status'),10)
        self.create_subscription(String,'/brain/health',lambda m:self.on_json(m,'module_health'),10)
        self.create_subscription(String,'/brain/status',self.on_brain,10)
        self.create_subscription(String,'/brain/servo_metrics',self.on_servo,10)
        self.create_subscription(String,'/locomotion/rl_status',self.on_rl_status,10)
        self.create_subscription(String,'/motion/command_owner',self.on_command_owner,10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/viz/markers', latched)
        self.status_image = self.create_publisher(Image, '/viz/status_image', latched)
        self.status_text = self.create_publisher(String, '/viz/status_text', latched)
        self.actual = self.create_publisher(Path, '/viz/actual_path', latched)
        self.tool = self.create_publisher(Path, '/viz/tool_path', latched)
        self.plan_pub = self.create_publisher(Path, '/viz/global_plan', latched)
        self.planned_tool = self.create_publisher(Path, '/viz/planned_tool_path', latched)
        self.grid_pub = self.create_publisher(MarkerArray, '/viz/grids', latched)
        self.grids = {}
        self.grid_wall = {}
        self.create_subscription(OccupancyGrid, '/map', lambda m:self.on_grid(m, 'map'), latched)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m:self.on_grid(m, 'cost'), latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/task/state', self.on_stage, 10)
        self.create_subscription(String, '/task/metrics', self.on_metrics, 10)
        self.create_subscription(String, '/perception/labels', lambda m: setattr(self, 'labels', m.data), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_livo, 10)
        self.create_subscription(DisplayTrajectory, '/display_planned_path', self.on_arm_plan, 10)
        self.create_timer(0.2, self.publish)

    def on_metrics(self, msg):
        try:self.metrics=json.loads(msg.data)
        except ValueError:pass

    def on_manipulation(self,msg):
        try:self.manipulation=json.loads(msg.data);self.manipulation_wall=time.monotonic()
        except ValueError:return
        point=self.manipulation.get('object_xyz')
        if point is not None:self.append_point(self.object_points,point,.01)

    def on_brain(self,msg):
        try:self.brain_status=json.loads(msg.data)
        except ValueError:pass

    def on_json(self,msg,attribute):
        try:setattr(self,attribute,json.loads(msg.data))
        except ValueError:pass

    def on_servo(self,msg):
        try:self.servo_metrics=json.loads(msg.data);self.servo_wall=time.monotonic()
        except ValueError:pass

    def on_rl_status(self,msg):
        try:self.rl_status=json.loads(msg.data)
        except ValueError:pass

    def on_command_owner(self,msg):
        try:
            data=json.loads(msg.data);self.command_owner=data.get('owner','unknown')
            if data.get('reason'):self.command_owner+=' ('+data['reason']+')'
        except ValueError:pass

    def on_grid(self, msg, name):
        # Ordinary geometry avoids indexed-texture shader failures on VcXsrv.
        now = time.monotonic()
        if now-self.grid_wall.get(name, -1) < 0.7:
            return
        self.grid_wall[name] = now
        values = np.asarray(msg.data)
        cells = np.flatnonzero(values >= (50 if name=='map' else 1))
        cells = cells[::max(1, math.ceil(len(cells)/16000))]
        m = Marker(header=msg.header, ns=name, id=0, type=Marker.CUBE_LIST, action=Marker.ADD)
        m.pose = msg.info.origin
        m.pose.position.z += 0.012 if name=='map' else 0.025
        m.scale.x = m.scale.y = msg.info.resolution
        m.scale.z = 0.012
        m.color.r, m.color.g, m.color.b, m.color.a = (0.38,0.51,0.65,0.65) if name=='map' else (1.,0.27,0.12,0.3)
        m.points = [Point(x=float((i%msg.info.width+.5)*msg.info.resolution), y=float((i//msg.info.width+.5)*msg.info.resolution),z=0.) for i in cells]
        self.grids[name] = m
        self.grid_pub.publish(MarkerArray(markers=list(self.grids.values())))

    @staticmethod
    def stage_label(text):
        if '路线检查点' in text:return 'Route checkpoint '+text.split()[-1]
        # RViz's bundled font atlas cannot reliably render Chinese glyphs.
        for token,label in [('TASK_SUCCESS','TASK_SUCCESS'),('TASK_FAILED','TASK_FAILED'),('等待','Waiting for systems'),('就绪','System readiness'),('收回','Retracting arm'),('搬运','Carrying'),('抬起完成','Navigating to place'),('Nav2','Nav2 navigation'),('接近位','Aligning approach'),('预抓取','Pre-grasp'),('下降','Lowering arm'),('attached','Object attached'),('抬起','Lifting object'),('放置','Placing object'),('released','Object released'),('detach','Object released'),('真值','Checking final position')]:
            if token in text:return label
        return text if text.isascii() else 'Manipulation in progress (see task.log)'

    def on_stage(self, msg):
        self.stage = msg.data
        self.last_task_wall = time.monotonic()

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_livo(self, msg):
        self.livo_last = msg
        self.livo_wall = time.monotonic()

    def on_plan(self, msg):
        self.plan = msg
        self.plan_pub.publish(msg)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall = time.monotonic()
        if self.sim_previous is not None and wall-self.wall_previous > 1.0:
            self.rtf = (now-self.sim_previous)/(wall-self.wall_previous)
            self.sim_previous, self.wall_previous = now, wall
        elif self.sim_previous is None:
            self.sim_previous, self.wall_previous = now, wall
        p = self.pose.position
        self.append_point(self.base_points, (p.x, p.y, 0.04), 0.015)

    @staticmethod
    def append_point(points, p, spacing):
        if not points or np.linalg.norm(np.array(p)-points[-1]) > spacing:
            points.append(np.asarray(p))

    def to_world(self, p):
        q = self.pose.orientation
        from qda_task.pick_place_node import _quat_to_rot
        r = _quat_to_rot(q.x, q.y, q.z, q.w)
        b = self.pose.position
        return np.asarray(p) @ r.T + np.array([b.x, b.y, b.z])

    def path(self, points):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for xyz in points:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, xyz)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def on_arm_plan(self, msg):
        if self.pose is None:
            return
        names = [f'left_joint_link{i}' for i in range(1, 8)]
        rows = []
        for tr in msg.trajectory:
            jt = tr.joint_trajectory
            if not all(n in jt.joint_names for n in names):
                continue
            indices = [jt.joint_names.index(n) for n in names]
            rows.extend([[p.positions[i] for i in indices] for p in jt.points])
        if rows:
            pad, _, _ = _aubo_fk(np.array(rows), _LEFT_MOUNT, np.array(LEFT_ARM_BASE))
            self.planned_tool.publish(self.path(self.to_world(pad)))

    def marker(self, ident, kind, xyz, scale, rgba, text=''):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'algorithm_view'
        m.id, m.type, m.action = ident, kind, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        m.text = text
        return m

    def publish(self):
        if self.pose is None:
            return
        names = [f'left_joint_link{i}' for i in range(1, 8)]
        if all(n in self.joints for n in names):
            pad, _, _ = _aubo_fk(np.array([[self.joints[n] for n in names]]), _LEFT_MOUNT, np.array(LEFT_ARM_BASE))
            self.append_point(self.tool_points, self.to_world(pad)[0], 0.008)
        self.actual.publish(self.path(self.base_points))
        self.tool.publish(self.path(self.tool_points))
        markers = []
        if self.manipulation.get('object_xyz'):
            payload=self.marker(500,Marker.CUBE,self.manipulation['object_xyz'],(.04,.04,.04),(.95,.05,.03,1.))
            q=self.manipulation.get('object_quat_wxyz',[1,0,0,0])
            payload.pose.orientation.w,payload.pose.orientation.x,payload.pose.orientation.y,payload.pose.orientation.z=map(float,q)
            markers.append(payload)
            trail=self.marker(501,Marker.LINE_STRIP,(0,0,0),(.012,0,0),(1.,.25,.1,.85))
            trail.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in self.object_points]
            markers.append(trail)
        # These wireframes show the configured reference scene, not detections.
        for i, (name, pos, size, _) in enumerate(planning_boxes()):
            markers.append(self.marker(10+i, Marker.CUBE, pos, size, (0.35, 0.55, 0.72, 0.16)))
            markers.append(self.marker(100+i, Marker.TEXT_VIEW_FACING, (pos[0], pos[1], pos[2]+size[2]/2+0.12), (0, 0, 0.10), (0.85, 0.91, 1, 1), name))
        if EXPANDED:
            for index,(name,points) in enumerate(ROUTE_OPTIONS.items()):
                color=(.2,.65,1.,.6) if name=='north' else (1.,.7,.12,.6)
                line=self.marker(200+index,Marker.LINE_STRIP,(0,0,0),(.035,0,0),color)
                line.points=[Point(x=float(x),y=float(y),z=.035) for x,y,_ in points]
                markers.append(line)
                for k,(x,y,yaw) in enumerate(points):
                    active=all(p in ROUTE_WAYPOINTS for p in points)
                    markers.append(self.marker(220+index*10+k,Marker.TEXT_VIEW_FACING,(x,y,.2),(0,0,.12),color,f'{name} {k+1}'+(' [selected]' if active else ' [option]')))
            c,s=math.cos(PLACE_YAW),math.sin(PLACE_YAW)
            ax,ay=APPROACH_IN_BASE
            poses=[*ROUTE_WAYPOINTS,(PLACE_MARKER_XYZ[0]-c*ax+s*ay,PLACE_MARKER_XYZ[1]-s*ax-c*ay,PLACE_YAW)]
            for k,(x,y,yaw) in enumerate(poses):
                arrow=self.marker(300+k,Marker.ARROW,(x,y,.08),(.40,.06,.06),(.3,1.,.6,.85))
                arrow.pose.orientation.z=math.sin(yaw/2);arrow.pose.orientation.w=math.cos(yaw/2)
                markers.append(arrow)
        livo = 'waiting'
        if self.livo_last and time.monotonic()-self.livo_wall < 3:
            livo = 'receiving (not navigation odometry)'
        status = f'{self.backend.upper()} | kinematic comparison\nRTF {self.rtf:.2f} | cmd {self.cmd.linear.x:+.2f}, {self.cmd.linear.y:+.2f} m/s\nLIVO: {livo}\nTask: {self.stage_label(self.stage)}\nPerception: {self.labels[:70]}'
        if self.rl_status:
            r=self.rl_status
            for i,(xyz,contact) in enumerate(zip(r.get('foot_positions_world',[]),r.get('foot_contacts',[]))):
                markers.append(self.marker(400+i,Marker.SPHERE,xyz,(.09,.09,.09),(.2,.95,.35,.9) if contact else (1.,.3,.15,.7)))
            for i,(key,color) in enumerate((('desired_velocity',(.2,.7,1.,1.)),('actual_body_velocity',(1.,.7,.15,1.)))):
                velocity=r.get(key,[0.,0.,0.])
                arrow=self.marker(410+i,Marker.ARROW,(0,0,0),(.025,.06,.07),color)
                points=self.to_world([[0.,0.,.8+i*.1],[velocity[0]*3,velocity[1]*3,.8+i*.1]])
                arrow.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points]
                markers.append(arrow)
            status=(f'MUJOCO | RL + physical PD (experimental)\nRTF {self.rtf:.2f} | owner {self.command_owner}'
                    f'\nTilt {math.degrees(r.get("tilt_rad",0)):.1f} deg | height {r.get("height",0):.3f} m'
                    f'\nPolicy ticks {r.get("policy_ticks",0)} | fault {r.get("fault") or "none"}'
                    '\nBlue: command | gold: simulated actual velocity | green feet: contact'
                    f'\nNavigation odometry: SIMULATION TRUTH\nGrasp: {r.get("grasp","unknown")}'
                    f'\nTask: {self.manipulation.get("stage",self.stage_label(self.stage))}\nPerception: {self.labels[:65]}')
        travelled=float(np.linalg.norm(np.diff(np.asarray(self.base_points),axis=0),axis=1).sum()) if len(self.base_points)>1 else 0.
        status+=f'\nScenario: {SCENARIO} | observed base path {travelled:.2f} m'
        if self.metrics:status+=f'\nPlan calls/failures: {self.metrics.get("planning_calls",0)}/{self.metrics.get("planning_failures",0)} | vision fallback {len(self.metrics.get("perception_fallbacks",[]))}'
        if self.brain_status:
            b=self.brain_status
            status+=f'\nBrain skill: {b.get("last_skill")} | {b.get("last_status")} | error {b.get("last_error") or "none"}'
        if self.manipulation:
            g=self.manipulation
            status+=f'\nPhysical task: {g.get("stage")} | carrying {g.get("carrying")} | {g.get("error") or "no error"}'
        if self.servo_metrics:
            s=self.servo_metrics;fresh='live' if time.monotonic()-self.servo_wall<5 else 'last observation'
            status+=f'\nIBVS {s.get("arm")} | {s.get("error_px",0):.2f} px | {s.get("depth_m",0):.3f} m | {fresh}'
            status+=f' | {s.get("feature_source","aruco")}'
        if self.mission_status:
            m=self.mission_status
            status+=f'\nLast mission: {m.get("status")} | {m.get("completed_steps",0)}/{m.get("total_steps",0)} | {m.get("stage","idle")}'
        if self.module_health:
            stale=[name for name,row in self.module_health.get('topics',{}).items() if not row.get('fresh')]
            status+=f'\nModule freshness: {"OK" if not stale else ", ".join(stale)}'
        color = (0.3, 1, 0.65, 1) if 'TASK_SUCCESS' in self.stage else ((1, 0.3, 0.25, 1) if 'FAILED' in self.stage else (0.9, 0.94, 1, 1))
        self.status_text.publish(String(data=status))
        # Long world-space text covers paths and is distorted by the font atlas
        # on some X servers. Put readable diagnostics in a dedicated image dock.
        old=self.marker(1, Marker.TEXT_VIEW_FACING, (0,0,0), (0,0,.1), color, '')
        old.action=Marker.DELETE;markers.append(old)
        r=self.rl_status;b=self.brain_status;m=self.mission_status;s=self.servo_metrics
        lines=[f'{self.backend.upper()} | {"RL + physical PD (experimental)" if r else "kinematic comparison"}',
               f'RTF {self.rtf:.2f} | owner: {self.command_owner}',
               f'Fault: {r.get("fault") or "none"} | truth odometry',
               f'Last mission: {m.get("status","IDLE")}  {m.get("completed_steps",0)}/{m.get("total_steps",0)}',
               f'Stage: {self.manipulation.get("stage",m.get("stage","idle"))}',
               f'Skill: {b.get("last_skill","none")} | {b.get("last_status","idle")}',
               f'Error: {b.get("last_error") or "none"}',
               f'IBVS {s.get("arm","-")}: {s.get("error_px",0):.2f} px | {"live" if time.monotonic()-self.servo_wall<5 else "last observation"}',
               f'Features: {s.get("feature_source","-")}',
               f'Physical pick/place: {self.manipulation.get("stage","waiting")}',
               f'Payload: {"held" if self.manipulation.get("carrying") else "released"} | {"live" if time.monotonic()-self.manipulation_wall<5 else "stale"}',
               'Pad normal force (N): '+str([round(v,1) for v in self.manipulation.get('pad_normal_force_n',[])]),
               f'Max slip: {1000*self.manipulation.get("max_slip_m",0):.1f} mm | pose: simulation truth',
               'Physical error: '+(self.manipulation.get('error') or 'none')]
        topics=self.module_health.get('topics',{})
        stale=[name for name,entry in topics.items() if not entry.get('fresh')]
        lines.append('Modules: '+(', '.join(stale) if stale else 'fresh' if topics else 'waiting'))
        import textwrap
        lines=[part for line in lines for part in textwrap.wrap(line,width=55)]
        canvas=np.full((max(390,55+len(lines)*29),780,3),(39,29,23),dtype=np.uint8)
        for row,line in enumerate(lines):
            cv2.putText(canvas,line,(20,35+row*29),cv2.FONT_HERSHEY_SIMPLEX,.67,
                        (145,220,185) if row==0 else (228,232,236),1,cv2.LINE_AA)
        message=Image();message.header.stamp=self.get_clock().now().to_msg();message.header.frame_id='odom'
        message.height,message.width=canvas.shape[:2];message.encoding='bgr8';message.step=message.width*3
        message.data=canvas.tobytes();self.status_image.publish(message)
        self.markers.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = AlgorithmView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
