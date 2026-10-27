"""Read-only ROS 2 graph inventory, interface connectivity and message samples.

Run inside the ROS container on the same domain as the simulation. This probe
does not publish control commands or submit tasks. Graph presence is distinct
from message activity and from algorithm/task acceptance.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.action.graph import (get_action_client_names_and_types_by_node,
                               get_action_server_names_and_types_by_node)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


SAMPLE_TOPICS = (
    '/clock', '/odom', '/joint_states', '/imu', '/scan',
    '/livox/lidar_PointCloud2', '/camera/image_raw', '/camera/depth/image_raw',
    '/cameras/left_wrist/image_raw', '/cameras/right_wrist/image_raw',
    '/mapping/rgbd_observation', '/mapping/colored_global', '/mapping/color_status',
    '/map', '/locomotion/rl_status', '/motion/command_owner', '/motion/arm_modes',
    '/manipulation/status', '/perception/labels', '/perception/pick_pose',
    '/perception/place_pose', '/perception/debug', '/brain/status',
    '/brain/health', '/brain/mission_status', '/brain/decision_trace',
    '/brain/servo_metrics', '/viz/status_image', '/cmd_vel', '/cmd_vel_nav',
    '/servo_left/status', '/servo_right/status', '/monitored_planning_scene',
)


def fqn(name, namespace):
    return (namespace.rstrip('/') + '/' + name)


def pairs(rows):
    return [{'name': name, 'types': types} for name, types in sorted(rows)]


def brief(msg):
    if hasattr(msg, 'data') and isinstance(msg.data, str):
        try:
            return json.loads(msg.data)
        except ValueError:
            return msg.data[:1000]
    result = {'type': type(msg).__name__}
    if hasattr(msg, 'header'):
        result.update(frame=msg.header.frame_id,
                      stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    for field in ('width', 'height', 'encoding', 'point_step'):
        if hasattr(msg, field):
            result[field] = getattr(msg, field)
    if hasattr(msg, 'clock'):
        result['sim_time'] = msg.clock.sec + msg.clock.nanosec * 1e-9
    if type(msg).__name__ == 'JointState':
        result['joint_count'] = len(msg.name)
    if type(msg).__name__ == 'Twist':
        result['velocity'] = [msg.linear.x, msg.linear.y, msg.angular.z]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/ws/output/ros2_graph')
    parser.add_argument('--seconds', type=float, default=18.)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('qda_graph_audit')
    try:
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        nodes = []
        names = node.get_node_names_and_namespaces()
        duplicates = {k: v for k, v in Counter(fqn(n, ns) for n, ns in names).items() if v > 1}
        for name, ns in sorted(set(names), key=lambda p: fqn(*p)):
            if name == node.get_name():
                continue
            row = dict(name=fqn(name, ns))
            for key, method in (
                ('publishers', node.get_publisher_names_and_types_by_node),
                ('subscribers', node.get_subscriber_names_and_types_by_node),
                ('services', node.get_service_names_and_types_by_node),
                ('clients', node.get_client_names_and_types_by_node),
            ):
                row[key] = pairs(method(name, ns))
            row['action_servers'] = pairs(get_action_server_names_and_types_by_node(node, name, ns))
            row['action_clients'] = pairs(get_action_client_names_and_types_by_node(node, name, ns))
            nodes.append(row)
        topics = []
        for topic, types in sorted(node.get_topic_names_and_types()):
            pubs = node.get_publishers_info_by_topic(topic)
            subs = node.get_subscriptions_info_by_topic(topic)
            def endpoints(infos):
                return sorted(set(fqn(i.node_name, i.node_namespace) for i in infos
                                  if i.node_name != node.get_name()))
            topics.append(dict(name=topic, types=types, publishers=endpoints(pubs), subscribers=endpoints(subs)))
        samples = {}
        subscriptions = []
        topic_types = dict(node.get_topic_names_and_types())
        for topic in SAMPLE_TOPICS:
            row = dict(count=0, last=None)
            samples[topic] = row
            types = topic_types.get(topic)
            if not types:
                row['note'] = 'not discovered'
                continue
            # Best effort receives both sensor QoS and reliable publications.
            latched = topic in ('/map', '/mapping/colored_global', '/mapping/color_status', '/viz/status_image')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
            def received(msg, current=row):
                current['count'] += 1
                current['last'] = brief(msg)
                if current['count'] == 1:
                    current['first'] = current['last']
            subscriptions.append(node.create_subscription(get_message(types[0]), topic, received, qos))
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        elapsed = time.monotonic() - start
        # Freeze the sample before service queries spin the executor again.
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        for row in samples.values():
            row['received_hz_wall'] = round(row['count'] / elapsed, 3)
        model = {}
        try:
            with urllib.request.urlopen('http://127.0.0.1:8088/health', timeout=3) as response:
                model = json.load(response)
        except Exception as exc:
            model = dict(error=str(exc))
        lifecycle = {}
        from lifecycle_msgs.srv import GetState
        for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                     'local_costmap/local_costmap', 'global_costmap/global_costmap'):
            client = node.create_client(GetState, '/' + name + '/get_state')
            if client.wait_for_service(timeout_sec=1):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                if future.done() and future.result() is not None:
                    lifecycle[name] = future.result().current_state.label
                else:
                    lifecycle[name] = 'no response'
            else:
                lifecycle[name] = 'service unavailable'
            node.destroy_client(client)
        report = dict(captured_at=datetime.now(timezone.utc).isoformat(),
                      source='live rclpy graph and actual received ROS messages',
                      ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                      probe_excluded=True, observation_wall_seconds=elapsed,
                      nodes=nodes, duplicate_node_names=duplicates, topics=topics,
                      services=pairs((name, types) for name, types in node.get_service_names_and_types()
                                     if not name.startswith('/qda_graph_audit/')),
                      samples=samples, nav2_lifecycle=lifecycle, model_health=model)
        path = out / 'ros2-live-graph.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        (out / 'nodes.txt').write_text('\n'.join(n['name'] for n in nodes) + '\n', encoding='utf-8')
        print(json.dumps(dict(report=str(path), nodes=len(nodes), topics=len(topics),
                              lifecycle=lifecycle, model_health=model,
                              sampled_topics_with_messages=sum(s['count'] > 0 for s in samples.values())), ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
