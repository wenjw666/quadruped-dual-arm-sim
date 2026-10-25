"""Bounded end-to-end check against the running factory launch (ROS domain 8).

Uses real ROS actions and sensor streams; never writes simulation positions.
Navigation odometry remains simulation truth and is labelled in the report.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, Image, LaserScan, PointCloud2, CameraInfo
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from qda_interfaces.srv import Attach


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--goal',nargs=3,type=float,default=[-.2,1.,0.])
    parser.add_argument('--output',default='/ws/output/v2_scene/integration_test.json')
    parser.add_argument('--wall-timeout',type=float,default=600.)
    options=parser.parse_args()
    rclpy.init();node=rclpy.create_node('v2_scene_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    start=time.monotonic();latest={};counts={};states=[];path=[];owner_history=[];active=[]
    tf=Buffer();listener=TransformListener(tf,node)
    report={'passed':False,'localization':'simulation_truth','physical_grasp_validated':False,'runtime_ready':False}
    def record(key,msg):
        latest[key]=msg;counts[key]=counts.get(key,0)+1
        if key=='status':states.append(json.loads(msg.data))
        if key=='owner':
            owner=json.loads(msg.data)['owner']
            if not owner_history or owner_history[-1]!=owner:owner_history.append(owner)
        if key=='odom':path.append([msg.pose.pose.position.x,msg.pose.pose.position.y])
    for typ,topic,key,qos in [
        (String,'/locomotion/rl_status','status',10),(String,'/motion/command_owner','owner',10),
        (Odometry,'/odom','odom',10),(JointState,'/joint_states','joints',10),
        (Image,'/camera/image_raw','camera',qos_profile_sensor_data),(LaserScan,'/scan','scan',qos_profile_sensor_data),
        (PointCloud2,'/v2/filtered_cloud','filtered_cloud',qos_profile_sensor_data),
        (NavPath,'/plan','plan',10),(OccupancyGrid,'/map','map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
        node.create_subscription(typ,topic,lambda msg,k=key:record(k,msg),qos)
    for camera in ('c0','left_wrist','right_wrist'):
        node.create_subscription(Image,f'/cameras/{camera}/image_raw',lambda msg,k=camera:record(k,msg),qos_profile_sensor_data)
        node.create_subscription(CameraInfo,f'/cameras/{camera}/camera_info',lambda msg,k=camera:record(k+'_info',msg),qos_profile_sensor_data)
    def wait(predicate,seconds=60.):
        deadline=min(start+options.wall_timeout,time.monotonic()+seconds)
        while not predicate() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if states and states[-1]['fault']:raise AssertionError(states[-1]['fault'])
        if not predicate():raise TimeoutError('ROS integration stage exceeded deadline')
    def result(future,seconds=60.):
        wait(future.done,seconds);return future.result()
    hold=node.create_client(SetBool,'/motion/hold')
    nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    arms={s:ActionClient(node,FollowJointTrajectory,f'/{s}_arm_controller/follow_joint_trajectory') for s in ('left','right')}
    try:
        wait(lambda:all(k in latest for k in ('status','odom','joints','camera','scan','map','filtered_cloud',
            'c0','left_wrist','right_wrist','c0_info','left_wrist_info','right_wrist_info')) and states[-1]['ready'],120)
        import numpy as np
        from PIL import Image as PilImage
        camera_report={}
        for camera,parent in (('c0','base_link'),('left_wrist','left_link7'),('right_wrist','right_link7')):
            frame=camera+'_optical_frame';msg=latest[camera];info=latest[camera+'_info']
            transform=tf.lookup_transform(parent,frame,rclpy.time.Time())
            pixels=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            assert msg.header.frame_id==frame and info.header.frame_id==frame and info.k[0]>0
            assert msg.width==info.width and msg.height==info.height and pixels.std()>5,'Invalid camera image'
            PilImage.fromarray(pixels).save(str(Path(options.output).with_name(camera+'_verified.png')))
            camera_report[camera]={'frame':frame,'parent':parent,'image_std':float(pixels.std()),'width':msg.width,'height':msg.height}
        report['aux_cameras']=camera_report
        planning=node.create_client(GetPlanningScene,'/get_planning_scene');wait(planning.service_is_ready)
        request=GetPlanningScene.Request();request.components.components=PlanningSceneComponents.OCTOMAP
        scene=result(planning.call_async(request)).scene
        report['octomap']={'frame':scene.world.octomap.header.frame_id,'resolution':scene.world.octomap.octomap.resolution,
                           'data_bytes':len(scene.world.octomap.octomap.data)}
        assert report['octomap']['data_bytes']>0,'OctoMap has no occupancy data'
        assert report['octomap']['frame']=='odom','OctoMap must stay world-fixed as the base moves'
        publishers=node.get_publishers_info_by_topic('/cmd_vel')
        assert len(publishers)==1 and publishers[0].node_name=='qda_command_mux',[(p.node_name,p.node_namespace) for p in publishers]
        report['exclusive_command_publisher']=True
        print('Sensors ready; exclusive command publisher verified',flush=True)
        assert not node.get_publishers_info_by_topic('/leg_controller/commands'),'Legacy leg controller still active'
        wait(lambda:hold.service_is_ready())
        assert result(hold.call_async(SetBool.Request(data=True))).success
        # Both arms execute a small sequential motion, then return home.
        arm_results=[]
        for side,client in arms.items():
            wait(client.server_is_ready)
            names=[f'{side}_joint_link{i}' for i in range(1,8)]
            current=dict(zip(latest['joints'].name,latest['joints'].position))
            home=[current[n] for n in names];target=home.copy();target[0]+=.08 if side=='left' else -.08
            for positions in (target,home):
                goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
                goal.trajectory.points=[JointTrajectoryPoint(positions=positions,time_from_start=Duration(sec=3))]
                handle=result(client.send_goal_async(goal));assert handle.accepted
                active.append(handle);answer=result(handle.get_result_async(),90);active.remove(handle)
                arm_results.append({'side':side,'status':answer.status,'error_code':answer.result.error_code,'detail':answer.result.error_string})
                print(json.dumps(arm_results[-1]),flush=True)
                assert answer.status==4 and answer.result.error_code==0,arm_results[-1]
        report['arm_actions']=arm_results
        assert result(hold.call_async(SetBool.Request(data=False))).success
        wait(nav.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='odom';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=options.goal[:2]
        goal.pose.pose.orientation.z=math.sin(options.goal[2]/2);goal.pose.pose.orientation.w=math.cos(options.goal[2]/2)
        handle=result(nav.send_goal_async(goal));assert handle.accepted;active.append(handle)
        print('Navigation goal accepted: '+str(options.goal),flush=True)
        answer=result(handle.get_result_async(),options.wall_timeout);active.remove(handle)
        position=latest['odom'].pose.pose.position
        error=math.hypot(position.x-options.goal[0],position.y-options.goal[1])
        report['navigation']={'goal':options.goal,'status':answer.status,'position_error_m':error,'path_points':counts.get('plan',0)}
        assert answer.status==4 and error<=.15,report['navigation']
        assert counts.get('plan',0)>0,'No actual global path'
        assert 'nav' in owner_history,'Navigation never owned velocity command'
        request.components.components=PlanningSceneComponents.ROBOT_STATE
        robot=result(planning.call_async(request)).scene.robot_state
        root=robot.multi_dof_joint_state
        assert 'world_joint' in root.joint_names,'MoveIt is missing the floating base state'
        translation=root.transforms[root.joint_names.index('world_joint')].translation
        report['moveit_base_error_m']=math.hypot(translation.x-position.x,translation.y-position.y)
        assert report['moveit_base_error_m']<.05,'MoveIt base pose is not tracking the physics TF'
        report['passed']=True
    except Exception as exc:
        report['failure']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        for handle in active:handle.cancel_goal_async()
        if hold.service_is_ready():
            future=hold.call_async(SetBool.Request(data=True))
            end=time.monotonic()+3
            while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        report.update(wall_seconds=time.monotonic()-start,streams=counts,owners=owner_history,
            max_tilt_rad=max((s['tilt_rad'] for s in states),default=None),
            min_height_m=min((s['height'] for s in states),default=None),
            faults=sorted(set(s['fault'] for s in states if s['fault'])),trajectory_xy=path,
            final_status=states[-1] if states else None)
        Path(options.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='trajectory_xy'},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
