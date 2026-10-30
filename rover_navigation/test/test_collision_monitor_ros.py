"""Native Humble monitor: slow for obstacles and resume without HOLD/goal cancel."""

import math
import json
from pathlib import Path
import subprocess
import time
import yaml
from rover_navigation.path_validation import apply_obstacle_clearance

from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rover_navigation.pixhawk_command_adapter_node import PixhawkCommandAdapter


def test_native_collision_slowdown_clear_and_missing_scan(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '89')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces>'
                       '<AllowMulticast>false</AllowMulticast></General>'
                       '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<Peers><Peer Address="localhost"/></Peers></Discovery>'
                       '</Domain></CycloneDDS>')
    config = Path(__file__).parents[1] / 'config/collision_monitor.yaml'
    nav = yaml.safe_load((config.parent / 'nav2_rtab.yaml').read_text())
    apply_obstacle_clearance(nav, lidar_obstacles=True)
    costmap = nav['local_costmap']['local_costmap']['ros__parameters']
    padding = costmap['footprint_padding']
    padded = [(x + math.copysign(padding, x), y + math.copysign(padding, y))
              for x, y in json.loads(costmap['footprint'])]
    log = (tmp_path / 'monitor.log').open('w')
    monitor = subprocess.Popen(
        ['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor', '--ros-args',
         '--params-file', str(config)], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=['--ros-args', '-p', 'cmd_vel_topic:=/cmd_vel_vehicle',
                     '-p', 'require_scan:=true', '-p', 'maximum_reverse_speed_m_s:=0.4'])
    node = Node('synthetic_obstacle')
    adapter = PixhawkCommandAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(adapter)
    final = []
    node.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', final.append, 10)
    commands = node.create_publisher(Twist, '/cmd_vel_guarded', 10)
    scans = node.create_publisher(LaserScan, '/scan_front', 10)
    footprints = node.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
    outputs = []
    node.create_subscription(Twist, '/cmd_vel_vehicle', outputs.append, 10)
    dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
    tf.transform.translation.x, tf.transform.translation.z = 0.32, 0.18
    tf.transform.rotation.w = 1.0
    tf.header.stamp = node.get_clock().now().to_msg()
    static.sendTransform(tf)
    client = node.create_client(ChangeState, '/collision_monitor/change_state')

    def run(duration, obstacle=False, scan_enabled=True, distance=None, bearing=0.0,
            speed=0.35, yaw_rate=0.0):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
            tf.transform.rotation.w = 1.0
            dynamic.sendTransform(tf)
            footprint = PolygonStamped()
            footprint.header.stamp, footprint.header.frame_id = stamp, 'base_link'
            footprint.polygon.points = [Point32(x=x, y=y) for x, y in padded]
            footprints.publish(footprint)
            if scan_enabled:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                scan.angle_min, scan.angle_max = bearing - 0.1, bearing + 0.1
                scan.angle_increment = 0.02
                scan.range_min, scan.range_max = 0.15, 8.0
                scan.ranges = [distance if distance is not None else (
                    0.2 if obstacle else 3.0)] * 11
                scans.publish(scan)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = speed, yaw_rate
            commands.publish(cmd)
            until = time.monotonic() + 0.05
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)

    try:
        assert client.wait_for_service(timeout_sec=8)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=8)
            assert future.done() and future.result().success
        run(1.5)
        assert outputs and math.isclose(outputs[-1].linear.x, 0.35)
        # Laser is x=0.32, body front x=0.41: a 1.4 m return is ~1.31 m
        # ahead of the body. React while it is still more than 1 m ahead.
        run(1.0, distance=1.4)
        assert 0.0 < outputs[-1].linear.x < 0.35
        early_speed = outputs[-1].linear.x
        run(1.0, distance=1.1)
        assert 0.0 < outputs[-1].linear.x < early_speed
        # Equally distant lateral returns outside the forward swept footprint
        # must not trigger a blanket circular slowdown zone.
        run(1.0, distance=1.4, bearing=math.pi / 2)
        assert math.isclose(outputs[-1].linear.x, 0.35)
        run(1.0, distance=0.45, bearing=math.pi / 2)
        assert math.isclose(outputs[-1].linear.x, 0.35)
        outputs.clear()
        run(1.0, obstacle=True)
        assert outputs and outputs[-1].linear.x < 0.2
        run(1.0)
        assert math.isclose(outputs[-1].linear.x, 0.35)
        run(2.5, scan_enabled=False)
        # Native Humble ignores the missing scan; our adapter pauses without HOLD.
        assert final[-1].velocity.x == 0.0
        assert not adapter.hold_pending
        run(1.0)
        assert math.isclose(outputs[-1].linear.x, 0.35)
        assert math.isclose(final[-1].velocity.x, 0.35)
        # Rear costmap collision checks belong to RPP. The monitor/adapter
        # must preserve an already validated reverse command, including yaw.
        run(1.0, speed=-.35, yaw_rate=-.1)
        assert math.isclose(outputs[-1].linear.x, -.35)
        assert math.isclose(final[-1].velocity.x, -.35)
        # PositionTarget.yaw_rate is float32 on the ROS wire.
        assert math.isclose(final[-1].yaw_rate, -.1, abs_tol=1e-7)
        run(2.5, scan_enabled=False, speed=-.35, yaw_rate=-.1)
        assert final[-1].velocity.x == final[-1].yaw_rate == 0.
        run(1., speed=-.35, yaw_rate=-.1)
        assert math.isclose(final[-1].velocity.x, -.35)
    finally:
        executor.shutdown()
        adapter.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        monitor.terminate()
        monitor.wait(timeout=10)
        log.close()
