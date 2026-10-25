import math
import os
import subprocess
from xml.etree import ElementTree as ET

import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from qda_bringup.arms import inject_aubo
from qda_kinematics.kinematics import AUBO_LIMITS, LEG_JOINTS


def _enabled(text: str) -> bool:
    token = "".join(ch for ch in text.strip().lower() if ch.isascii() and ch.isalnum())
    return token in ("true", "1", "yes")


def _camera_fx(camera: str) -> float:
    """Match Gazebo: fx = (width/2) / tan(hfov/2) at 640x480."""
    hfov = {"d435": 1.211259, "d455": math.pi / 2}.get(camera, 1.20)
    return (640.0 / 2.0) / math.tan(hfov / 2.0)


def _mapping_nodes(move_group, locomotion, perception, dump, mapper, sim_time, lidar, camera, enable_livo=True):
    """Start FAST-LIVO2 once its package is installed. Mapping still runs without it."""
    nodes = [n for n in (move_group, locomotion, perception, dump, mapper) if n is not None]
    if not enable_livo:return nodes
    try:
        share = get_package_share_directory("fast_livo")
    except PackageNotFoundError:
        return nodes
    fx = _camera_fx(camera)
    overlay = {
        "camera": {"fx": fx, "fy": fx, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480},
    }
    from qda_kinematics.assembly import CAMERA_XYZ, CAMERA_RPY, LIDAR_XYZ, LIDAR_RPY
    from qda_kinematics.kinematics import _rpy_matrix
    import numpy as np
    rc = _rpy_matrix(*CAMERA_RPY) @ _rpy_matrix(-math.pi/2,0,-math.pi/2)
    rl = _rpy_matrix(*LIDAR_RPY)
    overlay['extrin_calib']={'extrinsic_T':list(LIDAR_XYZ),'extrinsic_R':rl.reshape(-1).tolist(),
        'Rcl':(rc.T@rl).reshape(-1).tolist(),'Pcl':(rc.T@(np.array(LIDAR_XYZ)-CAMERA_XYZ)).tolist()}
    if lidar == "mid360":
        overlay["common"] = {"lid_topic": "/livox/lidar"}
        overlay["preprocess"] = {
            "lidar_type": 1,
            "scan_line": 4,
            "scan_rate": 10,
            "blind": 0.20,
            "point_filter_num": 1,
            "feature_extract_enabled": False,
        }
    nodes.append(
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="log",
            parameters=[
                os.path.join(share, "config", "gazebo.yaml"),
                os.path.join(share, "config", "camera_gazebo.yaml"),
                overlay,
                sim_time,
            ],
        )
    )
    return nodes


def _nav2_nodes(sim_time, visualize=False, dynamic=False):
    """Holonomic MPPI on the live /map. One cmd_vel publisher, so the old sampler stays off."""
    share = get_package_share_directory("qda_bringup")
    params = os.path.join(share, "config", "nav2_quad.yaml")
    bt_dir = os.path.join(get_package_share_directory("nav2_bt_navigator"), "behavior_trees")
    bt_name = "navigate_to_pose_w_replanning_and_recovery.xml"
    if not os.path.exists(os.path.join(bt_dir, bt_name)):
        bt_name = "navigate_w_replanning_and_recovery.xml"
    specs = (
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_lifecycle_manager", "lifecycle_manager", "lifecycle_manager_navigation"),
    )
    nodes = []
    for package, executable, name in specs:
        extra = []
        if executable == 'controller_server':
            extra.append({'FollowPath.visualize': visualize, 'FollowPath.TrajectoryVisualizer.trajectory_step': 40,
                          'FollowPath.TrajectoryVisualizer.time_step': 4})
            if dynamic:extra.append({'FollowPath.vx_max':.15,'FollowPath.vy_max':.08,'FollowPath.wz_max':.25,
                                     'progress_checker.movement_time_allowance':60.,'failure_tolerance':5.})
        if executable=='planner_server' and dynamic:
            extra.append({'GridBased.plugin':'nav2_smac_planner/SmacPlanner2D','GridBased.tolerance':.15,
                          'GridBased.allow_unknown':True,'GridBased.max_iterations':1000000})
        if executable == "bt_navigator":
            extra.append({"default_nav_to_pose_bt_xml": os.path.join(bt_dir, bt_name)})
        nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=[params, sim_time, *extra],
                remappings=[('/cmd_vel','/cmd_vel_nav')] if dynamic else [],
            )
        )
    return nodes


def _urdf_to_spawn_sdf(urdf_path: str, sdf_path: str) -> None:
    """Gazebo only honors initial_position when it sits inside the joint axis."""
    converted = subprocess.run(
        ["gz", "sdf", "-p", urdf_path],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(converted.stdout)
    for joint in root.iter("joint"):
        initial = joint.find("initial_position")
        axis = joint.find("axis")
        if initial is None or axis is None:
            continue
        joint.remove(initial)
        axis.append(initial)
    xml = ET.tostring(root, encoding="unicode")
    with open(sdf_path, "w", encoding="utf-8") as handle:
        handle.write(xml)


def _arm_limits():
    limits = {}
    for prefix in ("left", "right"):
        for index in range(1, 8):
            lo, hi = AUBO_LIMITS[index - 1]
            limits[f"{prefix}_joint_link{index}"] = {
                "has_velocity_limits": True,
                "max_velocity": 1.5,
                "has_acceleration_limits": True,
                "max_acceleration": 2.0,
                "has_position_limits": True,
                "min_position": lo,
                "max_position": hi,
            }
    return limits


def _controllers():
    def arm(prefix):
        return {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"{prefix}_joint_link{index}" for index in range(1, 8)],
        }

    def grip(prefix):
        return {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [f"{prefix}_finger_left_joint", f"{prefix}_finger_right_joint"],
        }

    return {
        "controller_names": [
            "left_arm_controller",
            "right_arm_controller",
            "left_gripper_controller",
            "right_gripper_controller",
        ],
        "left_arm_controller": arm("left"),
        "right_arm_controller": arm("right"),
        "left_gripper_controller": grip("left"),
        "right_gripper_controller": grip("right"),
    }


def _ompl_group():
    return {"default_planner_config": "RRTConnect", "planner_configs": ["RRTConnect"]}


def _launch(context, *args, **kwargs):
    import json
    scenario=LaunchConfiguration('scenario').perform(context).strip()
    os.environ['QDA_SCENARIO']=scenario
    from qda_kinematics.scene import SPAWN_XYZ, scene_manifest
    from qda_bringup.world_sdf import world_xml
    backend = LaunchConfiguration('backend').perform(context).strip().lower()
    motion_mode=LaunchConfiguration('motion_mode').perform(context).strip()
    dynamic=motion_mode=='rl'
    if motion_mode not in ('kinematic','rl') or (dynamic and backend!='mujoco'):
        raise RuntimeError('RL motion mode requires the MuJoCo backend')
    if backend not in ('gazebo', 'mujoco'):
        raise RuntimeError('backend must be gazebo or mujoco')
    gui = _enabled(LaunchConfiguration("gui").perform(context))
    rviz = _enabled(LaunchConfiguration("rviz").perform(context))
    fast = _enabled(LaunchConfiguration("fast_sim").perform(context))
    run_task = _enabled(LaunchConfiguration("run_task").perform(context))
    use_neural = _enabled(LaunchConfiguration("use_neural").perform(context))
    lidar = LaunchConfiguration("lidar").perform(context).strip().lower()
    camera = LaunchConfiguration("camera").perform(context).strip().lower()
    if lidar not in ("mid360", "ray16"):
        raise RuntimeError(f"lidar 只能是 mid360 或 ray16，当前是 {lidar}")
    if camera not in ("d435", "d455", "generic"):
        raise RuntimeError(f"camera 只能是 d435、d455 或 generic，当前是 {camera}")
    print(f"传感器 雷达={lidar} 相机={camera} fx={_camera_fx(camera):.1f}", flush=True)

    custom_out=LaunchConfiguration('output_dir').perform(context).strip()
    out = custom_out or ('/ws/output' if backend == 'gazebo' else '/ws/output/mujoco')
    if not os.path.isabs(out):raise RuntimeError('output_dir must be an absolute path')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out,'scene_manifest.json'),'w',encoding='utf-8') as handle:
        manifest=scene_manifest()
        if dynamic:manifest.update(motion='RL_ONNX_free_base_PD',grasp='disabled unless explicitly enabled as virtual',
                                   localization='simulation_truth',planner='SmacPlanner2D',runtime_ready=False)
        json.dump(manifest,handle,indent=2)
    world_path = os.path.join(out, "pick_place.world")
    with open(world_path, "w", encoding="utf-8") as handle:
        handle.write(world_xml(fast))

    xacro_path = os.path.join(get_package_share_directory("qda_description"), "urdf", "qda_robot.urdf.xacro")
    robot_xml = inject_aubo(
        xacro.process_file(
            xacro_path,
            mappings={"lidar_model": lidar, "camera_model": camera},
        ).toxml()
    )
    urdf_path = os.path.join(out, "qda_robot.urdf")
    with open(urdf_path, "w", encoding="utf-8") as handle:
        handle.write(robot_xml)
    sdf_path = os.path.join(out, "qda_robot.sdf")
    if backend == 'gazebo':
        _urdf_to_spawn_sdf(urdf_path, sdf_path)
    else:
        if lidar != 'mid360':
            raise RuntimeError('The MuJoCo comparison backend currently supports lidar:=mid360')
        from qda_mujoco.model import build_model
        mjcf_path = build_model(urdf_path, world_path, os.path.join(out, 'factory.xml'))
        if dynamic:
            from qda_mujoco.dynamics import build_dynamic_model
            from pathlib import Path
            target=Path(out)/'factory_dynamic.xml'
            build_dynamic_model(mjcf_path,urdf_path,
                os.path.join(get_package_share_directory('qda_moveit_config'),'config','qda_robot.srdf'),target)
            mjcf_path=str(target)
            if _enabled(LaunchConfiguration('servo_fixture').perform(context)):
                from qda_mujoco.servo_fixture import add_servo_fixture
                add_servo_fixture(mjcf_path)
    srdf_path = os.path.join(get_package_share_directory("qda_moveit_config"), "config", "qda_robot.srdf")
    with open(srdf_path, encoding="utf-8") as handle:
        srdf_xml = handle.read()
    from qda_bringup.refined_assembly import refine_srdf
    srdf_xml = refine_srdf(srdf_xml, robot_xml)
    if dynamic:
        semantic=ET.fromstring(srdf_xml)
        ET.SubElement(semantic,'virtual_joint',name='world_joint',type='floating',parent_frame='odom',child_link='base_link')
        srdf_xml=ET.tostring(semantic,encoding='unicode')

    plugin_lib = os.path.join(get_package_prefix("qda_gazebo_plugins"), "lib")
    livox_lib = ""
    if lidar == "mid360":
        livox_lib = os.path.join(get_package_prefix("ros2_livox_simulation"), "lib")
    previous = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    plugin_path = ":".join(part for part in (livox_lib, plugin_lib, "/opt/ros/humble/lib", previous) if part)
    # Gazebo resolves package://pkg/meshes as model://pkg/meshes, so the
    # directory that contains each description package has to be on the path.
    share_parents = []
    for package in ("qda_description", "b2_description", "aubo_description", "go1_description"):
        try:
            share_parents.append(os.path.dirname(get_package_share_directory(package)))
        except PackageNotFoundError:
            pass
    model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    local_models = "/usr/share/gazebo-11/models"
    model_path = ":".join(part for part in (*share_parents, local_models, model_path) if part)

    sim_time = {"use_sim_time": True}
    gazebo_ros = get_package_share_directory("gazebo_ros")
    actions = [
        # Empty URI skips the online model database. That lookup hangs in this
        # container and blocks entity spawn, so controller_manager never appears.
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world_path, "verbose": "true", "pause": "true"}.items(),
        ),
    ]
    if backend == 'mujoco':
        actions = [SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
                   SetEnvironmentVariable('MUJOCO_GL', 'egl'),
                   Node(package='qda_mujoco', executable='mujoco_node', output='screen',
                        parameters=[{'model_path':mjcf_path, 'gui':gui,'motion_mode':motion_mode,
                            'allow_virtual_grasp':_enabled(LaunchConfiguration('allow_virtual_grasp').perform(context))}])]
        if dynamic:actions.append(Node(package='qda_bringup',executable='command_mux',parameters=[sim_time],output='screen'))
    if gui and backend == 'gazebo':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py"))
            )
        )
    if rviz:
        source_config = os.path.join(get_package_share_directory("qda_bringup"), "rviz", "factory.rviz")
        rviz_config = os.path.join(out, 'factory.rviz')
        with open(source_config, encoding='utf-8') as handle:
            config_text = handle.read()
        if scenario!='legacy':
            config_text=config_text.replace('Distance: 6.2','Distance: 11.8').replace('X: 1.1','X: 2.5').replace('Y: -0.25','Y: 0.35')
        with open(rviz_config, 'w', encoding='utf-8') as handle:
            handle.write(config_text)
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"robot_description": robot_xml, "use_sim_time": True}],
            )
        )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_xml, **sim_time}],
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "qda_robot",
            "-file",
            urdf_path,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            str(SPAWN_XYZ[2]),
            "-Y",
            "0",
        ],
    )
    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
        parameters=[sim_time],
    )
    spawner_rest = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "leg_controller",
            "left_arm_controller",
            "right_arm_controller",
            "left_gripper_controller",
            "right_gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        parameters=[sim_time],
    )
    moveit_params = {
        "robot_description": robot_xml,
        "robot_description_semantic": srdf_xml,
        "robot_description_kinematics": {
            "left_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
            },
            "right_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
            },
        },
        "robot_description_planning": {
            "joint_limits": _arm_limits(),
            "cartesian_limits": {"max_trans_vel": 0.5, "max_trans_acc": 1.0, "max_rot_vel": 1.2},
        },
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.2,
            "planner_configs": {"RRTConnect": {"type": "geometric::RRTConnect", "range": 0.0}},
            "left_arm": _ompl_group(),
            "right_arm": _ompl_group(),
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": False,
        "moveit_simple_controller_manager": _controllers(),
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 2.5,
            "allowed_goal_duration_margin": 1.0,
            "allowed_start_tolerance": 0.25,
        },
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        **sim_time,
    }
    if dynamic:
        moveit_params.update({'octomap_frame':'odom','octomap_resolution':.05,'sensors':['scene_lidar'],
            'scene_lidar.sensor_plugin':'occupancy_map_monitor/PointCloudOctomapUpdater',
            'scene_lidar.point_cloud_topic':'/livox/lidar_PointCloud2','scene_lidar.max_range':12.,
            'scene_lidar.point_subsample':2,'scene_lidar.padding_offset':.02,'scene_lidar.padding_scale':1.,
            'scene_lidar.max_update_rate':2.,'scene_lidar.filtered_cloud_topic':'/v2/filtered_cloud'})
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        parameters=[sim_time],
    )
    locomotion = Node(
        package="qda_locomotion",
        executable="locomotion_node",
        output="screen",
        parameters=[{"use_neural": use_neural, **sim_time}],
    )
    if dynamic:locomotion=None
    perception = Node(
        package="qda_perception",
        executable="detector_node",
        output="screen",
        parameters=[sim_time],
    )
    dump = Node(
        package="qda_perception",
        executable="frame_dump",
        output="screen",
        parameters=[sim_time, {'output_dir':out}],
    )
    mapper = Node(
        package="qda_perception",
        executable="mapper_node",
        output="screen",
        parameters=[
            sim_time,
            {
                "cloud_topic": "/livox/lidar_PointCloud2" if lidar == "mid360" else "/fast_livo/cloud",
                "publish_velodyne": lidar != "mid360",
            },
        ],
    )
    navigator = _nav2_nodes(sim_time, visualize=rviz,dynamic=dynamic)
    visualization = Node(package='qda_bringup', executable='visualization_node', output='screen',
                         parameters=[sim_time, {'backend':backend, 'output_dir':out,'task_enabled':run_task}])
    if backend == 'gazebo':
        actions.extend(
        [
            TimerAction(period=4.0, actions=[rsp, static_tf, spawn]),
            TimerAction(period=6.0, actions=[spawner_js, spawner_rest]),
            TimerAction(
                period=8.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty"],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=11.0,
                actions=_mapping_nodes(
                    move_group, locomotion, perception, dump, mapper, sim_time, lidar, camera
                )
                + navigator + [visualization],
            ),
        ]
    )
    else:
        actions.extend([rsp, static_tf,
            TimerAction(period=3.0, actions=_mapping_nodes(move_group, locomotion, perception, dump, mapper, sim_time, lidar, camera,enable_livo=not dynamic)+navigator+[visualization])])
    if dynamic and _enabled(LaunchConfiguration('brain').perform(context)):
        import yaml
        servo_config=os.path.join(get_package_share_directory('moveit_servo'),'config','panda_simulated_config.yaml')
        brain_nodes=[]
        for side in ('left','right'):
            with open(servo_config) as handle:params=yaml.safe_load(handle)
            params.update(command_in_type='speed_units',publish_period=.02,incoming_command_timeout=.35,
                is_primary_planning_scene_monitor=False,move_group_name=side+'_arm',planning_frame=side+'_arm_mount',
                ee_frame_name=side+'_gripper',robot_link_command_frame=side+'_gripper',
                command_out_topic=f'/{side}_arm_controller/servo_joint_trajectory',check_collisions=True,
                collision_check_rate=10.,self_collision_proximity_threshold=.01,scene_collision_proximity_threshold=.02)
            descriptions={k:moveit_params[k] for k in ('robot_description','robot_description_semantic','robot_description_kinematics','robot_description_planning')}
            brain_nodes.append(Node(package='moveit_servo',executable='servo_node_main',name='servo_'+side,
                parameters=[descriptions,{'moveit_servo':params,'octomap_resolution':.05,'octomap_frame':'odom'},sim_time],
                remappings=[('/planning_scene','/monitored_planning_scene')],output='log'))
        for executable in ('skill_server','brain_node','visual_servo'):
            brain_nodes.append(Node(package='qda_brain',executable=executable,parameters=[sim_time],output='screen'))
        actions.append(TimerAction(period=8.,actions=brain_nodes))
    if run_task:
        task = Node(
            package="qda_task",
            executable="pick_place_node",
            output="screen",
            parameters=[sim_time, {'output_dir':out,'backend':backend}],
            remappings=[('/cmd_vel','/cmd_vel_task')] if dynamic else [],
        )
        actions.append(TimerAction(period=14.0, actions=[task]))
        print("任务节点会在 14 秒后启动", flush=True)
    else:
        print("任务节点未启动", flush=True)
    # LEG_JOINTS is imported so a missing kinematics install fails at launch time.
    if len(LEG_JOINTS) != 12:
        raise RuntimeError("腿关节数量不正确")
    return [SetEnvironmentVariable('QDA_SCENARIO',scenario)] + actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument('backend', default_value='mujoco'),
            DeclareLaunchArgument('motion_mode',default_value='kinematic',description='kinematic comparison or RL free-base dynamics'),
            DeclareLaunchArgument('brain',default_value='false',description='Local skills, semantic gateway and guarded MoveIt Servo'),
            DeclareLaunchArgument('servo_fixture',default_value='false',description='Optical calibration boards, not grasp targets'),
            DeclareLaunchArgument('allow_virtual_grasp',default_value='false',description='Explicit comparison-only virtual grasp in RL mode'),
            DeclareLaunchArgument('scenario', default_value='auto', description='auto, north, south, side, or legacy'),
            DeclareLaunchArgument('output_dir', default_value='', description='Optional isolated output directory'),
            DeclareLaunchArgument("rviz", default_value="true", description="Algorithm dashboard; disable for headless benchmarks"),
            DeclareLaunchArgument("fast_sim", default_value="false"),
            DeclareLaunchArgument("run_task", default_value="true"),
            DeclareLaunchArgument("use_neural", default_value="false"),
            DeclareLaunchArgument("lidar", default_value="mid360"),
            DeclareLaunchArgument("camera", default_value="d435"),
            OpaqueFunction(function=_launch),
        ]
    )
