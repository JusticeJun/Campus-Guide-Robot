"""Native Nav2 tree/planner on a synthetic campus patch, with no vehicle interface."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowPath
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from rover_navigation.lidar_navigation import configure_obstacles
from rover_navigation.path_validation import apply_obstacle_clearance


def test_real_tree_plans_around_obstacles_and_supports_planned_gear_changes(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '90')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces>'
                       '<AllowMulticast>false</AllowMulticast></General>'
                       '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>'
                       '<Peers><Peer Address="localhost"/></Peers></Discovery>'
                       '</Domain></CycloneDDS>')
    root = Path(__file__).parents[1]
    params = yaml.safe_load((root / 'config/nav2_rtab.yaml').read_text())
    defaults_file = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    defaults = yaml.safe_load(defaults_file.read_text())
    configure_obstacles(params)
    apply_obstacle_clearance(params, lidar_obstacles=True)
    # Match navigation.launch: adding an observation checker must not change
    # ordinary RViz goal completion or existing reverse/obstacle behavior.
    controller = params['controller_server']['ros__parameters']
    controller['goal_checker_plugins'] = ['goal_checker', 'observation_goal_checker']
    controller['observation_goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker', 'stateful': True,
        'xy_goal_tolerance': .45, 'yaw_goal_tolerance': .25}
    for name in ('local_costmap', 'global_costmap'):
        params[name][name]['ros__parameters']['route_layer']['enabled'] = False
    bt = params['bt_navigator']['ros__parameters']
    bt['plugin_lib_names'] = defaults['bt_navigator']['ros__parameters']['plugin_lib_names']
    bt['plugin_lib_names'].append('localization_recovery_bt')
    bt['default_nav_to_pose_bt_xml'] = str(root / 'behavior_trees/navigate_lidar.xml')
    bt['default_nav_through_poses_bt_xml'] = str(
        root / 'behavior_trees/navigate_through_poses_ackermann.xml')
    config = tmp_path / 'nav.yaml'
    config.write_text(yaml.safe_dump(params))
    log = (tmp_path / 'nav.log').open('w')
    nav = subprocess.Popen(['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                            f'params_file:={config}', 'use_composition:=False'],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    buffer = subprocess.Popen(['ros2', 'run', 'rover_navigation', 'static_clearance'],
                              stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('synthetic_campus')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(OccupancyGrid, '/map', durable)
    odoms = node.create_publisher(Odometry, '/odom', 10)
    scans = node.create_publisher(LaserScan, '/scan_front', 10)
    ready = node.create_publisher(Bool, '/navigation/ready', 1)
    operator_pause = node.create_publisher(Bool, '/navigation/operator_paused', durable)
    commands = []
    node.create_subscription(Twist, "/cmd_vel_nav", commands.append, 10)
    plans = []
    node.create_subscription(NavPath, '/plan', plans.append, 10)
    local_plans = []
    node.create_subscription(NavPath, '/received_global_plan', local_plans.append, 10)
    static, dynamic = StaticTransformBroadcaster(node), TransformBroadcaster(node)
    transforms = []
    for parent, child, x in [('map', 'odom', 0.0), ('base_link', 'laser', 0.32)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.rotation.w, tf.transform.translation.x = 1.0, x
        tf.header.stamp = node.get_clock().now().to_msg()
        transforms.append(tf)
    static.sendTransform(transforms)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution, grid.info.width, grid.info.height = 0.05, 600, 400
    grid.info.origin.position.x = grid.info.origin.position.y = -10.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (600 * 400)
    maps.publish(grid)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    obstacle_present = True
    obstacle_half_rays = 28
    obstacle_range = 3.2
    robot_x = 0.0
    navigation_ready = True

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
            tf.transform.rotation.w = 1.0
            tf.transform.translation.x = robot_x
            dynamic.sendTransform(tf)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x = robot_x
            odoms.publish(odom)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser'
            scan.angle_min, scan.angle_max = -math.radians(100), math.radians(100)
            # At 5 cm grid resolution, use sub-cell ray spacing at 3.2 m.
            # Coarse 1-degree rays can leave raster cells between clearing rays.
            scan.angle_increment = math.pi / 720
            scan.range_min, scan.range_max = 0.15, 8.0
            scan.ranges = [
                obstacle_range
                if obstacle_present and abs(i - 400) <= obstacle_half_rays else float('inf')
                for i in range(801)]
            scans.publish(scan)
            ready.publish(Bool(data=navigation_ready))
            until = time.monotonic() + 0.05
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.005)

    def body_clearance(plan, points):
        clearance = float('inf')
        for item in plan.poses:
            p, q = item.pose.position, item.pose.orientation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            for px, py in points:
                dx, dy = px - p.x, py - p.y
                x = math.cos(yaw)*dx + math.sin(yaw)*dy
                y = -math.sin(yaw)*dx + math.cos(yaw)*dy
                clearance = min(clearance, math.hypot(max(-0.07-x, 0, x-0.41),
                                                      max(abs(y)-0.135, 0)))
        return clearance

    def scan_points():
        return [(0.32 + 3.2 * math.cos(math.radians(i / 4)),
                 3.2 * math.sin(math.radians(i / 4)))
                for i in range(-obstacle_half_rays, obstacle_half_rays + 1)]

    handle = None
    try:
        deadline = time.monotonic() + 90
        while not client.server_is_ready() and time.monotonic() < deadline:
            pump(0.2)
        assert client.server_is_ready(), (tmp_path / 'nav.log').read_text()[-4000:]
        pump(2.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.orientation.w = 8.0, 1.0
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 20
        while not future.done() and time.monotonic() < deadline:
            pump(0.1)
        assert future.done()
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        while not plans and time.monotonic() < deadline:
            pump(0.2)
        assert plans and plans[-1].poses, (tmp_path / 'nav.log').read_text()[-4000:]
        around = [p.pose.position for p in plans[-1].poses if 3.0 < p.pose.position.x < 4.0]
        assert around and min(abs(p.y) for p in around) > 0.5
        # Measure from the physical body (not its center) to each synthetic
        # obstacle return along the entire native Smac plan, including turns.
        clearance = body_clearance(plans[-1], scan_points())
        assert clearance >= 0.20, clearance
        assert not result.done()  # Goal remains active while the synthetic rover stands still.
        # A newly observed object spanning about 2.2 m needs a wider detour,
        # without relying on semantic person/static-object classification.
        obstacle_half_rays = 80
        plans.clear()
        pump(6.0)
        assert plans
        clearance = body_clearance(plans[-1], scan_points())
        assert clearance >= 0.20, clearance
        around = [p.pose.position for p in plans[-1].poses if 3.0 < p.pose.position.x < 4.0]
        assert around and max(abs(p.y) for p in around) > 1.2
        assert not result.done()
        obstacle_present = False
        plans.clear()
        pump(3.0)
        assert plans
        around = [p.pose.position for p in plans[-1].poses if 3.0 < p.pose.position.x < 4.0]
        assert around and max(abs(p.y) for p in around) < 0.3
        assert not result.done()
        # Actual NavigateToPose action survives a pause; its FollowPath child
        # stops and resumes without a new outer goal or localization restart.
        original_goal_id = list(handle.goal_id.uuid)
        operator_pause.publish(Bool(data=True))
        pump(1.0)
        commands.clear()
        pump(3.0)
        assert not result.done()
        assert all(abs(c.linear.x) < 1e-6 and abs(c.angular.z) < 1e-6 for c in commands)
        operator_pause.publish(Bool(data=False))
        commands.clear()
        pump(4.0)
        assert not result.done() and list(handle.goal_id.uuid) == original_goal_id
        assert any(c.linear.x > .01 for c in commands)
        # The same pipeline must retain the requested clearance for a statue
        # already present in the stored map, even with the live footprint .20.
        cells = list(grid.data)
        corners = []
        for row in range(192, 208):
            for col in range(266, 274):
                cells[row * 600 + col] = 100
                for ox, oy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                    corners.append((-10 + (col + ox) * .05, -10 + (row + oy) * .05))
        grid.data = cells
        maps.publish(grid)
        plans.clear()
        pump(6.0)
        assert plans
        clearance = body_clearance(plans[-1], corners)
        assert clearance >= 0.15, clearance
        assert not result.done()
        # Unknown strip spans the whole map: allowing unknown must permit a
        # route through it while the known statue above remains avoided.
        for row in range(400):
            for col in range(310, 316):
                cells[row * 600 + col] = -1
        grid.data = cells
        maps.publish(grid)
        plans.clear()
        pump(6.0)
        assert plans
        assert any(5.5 < p.pose.position.x < 5.8 for p in plans[-1].poses)
        assert body_clearance(plans[-1], corners) >= 0.15

        # A scan return in the padded body must not trap the global planner,
        # but must remain in the local collision map (not erased globally).
        cancelled = handle.cancel_goal_async()
        pump(0.5)
        assert cancelled.done()
        obstacle_range, obstacle_half_rays, obstacle_present = 0.20, 3, True
        pump(1.0)

        def cost_at(name, x, y):
            service = node.create_client(GetCostmap, f'/{name}_costmap/get_costmap')
            assert service.wait_for_service(timeout_sec=2)
            future = service.call_async(GetCostmap.Request())
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                pump(.1)
            assert future.done()
            msg = future.result().map
            meta = msg.metadata
            col = int((x - meta.origin.position.x) / meta.resolution)
            row = int((y - meta.origin.position.y) / meta.resolution)
            value = msg.data[row * meta.size_x + col]
            node.destroy_client(service)
            return value

        assert cost_at('global', .52, 0.0) < 253
        assert cost_at('local', .52, 0.0) >= 253
        # Footprint clearing belongs to the observation layer only. A saved
        # obstacle inside the same envelope must still block the global map.
        cells[201 * 600 + 205] = 100
        grid.data = cells
        maps.publish(grid)
        pump(1.0)
        assert cost_at('global', .275, .075) == 254

        # Exercise the actual plugin on a deliberately infeasible pursuit path.
        # It must emit a feasible arc and still stop for a real obstacle.
        grid.data = [0] * (600 * 400)
        maps.publish(grid)
        obstacle_present = False
        pump(2.0)
        follow = ActionClient(node, FollowPath, '/follow_path')
        assert follow.wait_for_server(timeout_sec=3)
        command = FollowPath.Goal()
        command.controller_id = 'FollowPath'
        command.goal_checker_id = 'goal_checker'
        command.path.header.frame_id = 'map'
        for x, y in [(0., 0.), (.1, -.3), (.3, -.6), (.6, -.7), (1., -.8), (2., -.9)]:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            command.path.poses.append(pose)
        commands.clear()
        future = follow.send_goal_async(command)
        pump(2.0)
        assert future.done() and future.result().accepted
        tracking = future.result().get_result_async()
        moving = [m for m in commands if m.linear.x > .01]
        assert len(moving) >= 5
        assert all(abs(m.angular.z) <= m.linear.x / 1.7 + 1e-6 for m in moving)
        assert any(abs(m.angular.z) > .05 for m in moving)
        # Return at x=0.82 m intersects the feasible forward swept footprint.
        obstacle_range, obstacle_half_rays, obstacle_present = .50, 3, True
        pump(2.0)
        assert tracking.done() and tracking.result().status == 6
        follow.destroy()

        # After making substantial progress, halt and resume FollowPath with
        # the SAME BT goal/cache. The retained route must start near the rover,
        # rather than replaying the prefix near the original departure point.
        obstacle_present = False
        pump(2.0)
        goal.pose.pose.position.x = 18.0
        future = client.send_goal_async(goal)
        pump(2.0)
        assert future.done() and future.result().accepted
        handle = future.result()
        result = handle.get_result_async()
        for step in range(1, 25):
            robot_x = step * .5
            pump(.25)
        navigation_ready = False
        pump(1.0)
        local_plans.clear()
        commands.clear()
        navigation_ready = True
        pump(2.0)
        assert not result.done()
        assert local_plans and all(p.poses for p in local_plans), (
            'Restarted tracking produced an empty local path',
            (tmp_path / 'nav.log').read_text()[-3000:])
        assert any(m.linear.x > .01 for m in commands)

        # Native Reeds-Shepp planning must actually emit a reverse segment,
        # not a large forward loop to a destination behind the parked rover.
        handle.cancel_goal_async()
        pump(.5)
        robot_x = 0.0
        pump(1.)
        plans.clear()
        commands.clear()
        goal.pose.pose.position.x = -3.0
        future = client.send_goal_async(goal)
        pump(4.)
        assert future.done() and future.result().accepted
        handle = future.result()
        assert plans and max(abs(p.pose.position.y) for p in plans[-1].poses) < .2
        assert any(m.linear.x < -.24 for m in commands)
        assert all(abs(m.linear.x) <= .35 + 1e-6 for m in commands)
        handle.cancel_goal_async()
        pump(.5)

        # A three-point path revisits the same coordinates in opposite gears.
        # Check both cusps and signed steering via the actual controller.
        follow = ActionClient(node, FollowPath, '/follow_path')
        command = FollowPath.Goal()
        command.controller_id = 'FollowPath'
        command.goal_checker_id = 'goal_checker'
        command.path.header.frame_id = 'map'
        for x, y in [(0., 0.), (-.1, .3), (-.3, .6), (-.6, .7), (-1., .8), (-2., .9)]:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.
            command.path.poses.append(p)
        commands.clear()
        future = follow.send_goal_async(command)
        pump(2.)
        assert future.done() and future.result().accepted
        backwards = [m for m in commands if m.linear.x < -.01]
        assert backwards and any(m.angular.z < -.05 for m in backwards)
        assert all(abs(m.angular.z) <= abs(m.linear.x)/1.7+1e-6 for m in backwards)
        future.result().cancel_goal_async()
        pump(.5)
        command.path.poses = []
        xs = [i*.05 for i in range(21)]
        xs += [1.-i*.05 for i in range(1, 17)]
        xs += [.2+i*.05 for i in range(1, 77)]
        for x in xs:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.orientation.w = x, 1.
            command.path.poses.append(p)
        future = follow.send_goal_async(command)
        commands.clear()
        pump(2.)
        assert future.done() and future.result().accepted
        gear_handle = future.result()
        assert any(m.linear.x > .1 for m in commands)
        for robot_x in (.5, .85, 1.01):
            pump(.3)
        commands.clear()
        pump(1.)
        assert any(m.linear.x < -.1 for m in commands)
        for robot_x in (.7, .4, .19):
            pump(.3)
        commands.clear()
        pump(1.)
        assert any(m.linear.x > .1 for m in commands)
        gear_handle.cancel_goal_async()
        pump(.5)

        # Ordinary arrival retains its 1.5 m tolerance; observation must
        # continue until .45 m, using the same native controller server.
        robot_x = 0.
        pump(.5)
        command.path.poses = []
        for x in (0., .25, .5, .75, 1.):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.orientation.w = x, 1.
            command.path.poses.append(p)
        command.goal_checker_id = 'goal_checker'
        future = follow.send_goal_async(command)
        pump(.5)
        completion = future.result().get_result_async()
        pump(.5)
        assert completion.done() and completion.result().status == 4
        command.goal_checker_id = 'observation_goal_checker'
        future = follow.send_goal_async(command)
        pump(.5)
        completion = future.result().get_result_async()
        pump(.5)
        assert not completion.done()
        robot_x = .8
        pump(1.)
        assert completion.done() and completion.result().status == 4
        command.goal_checker_id = 'goal_checker'

        # Rear obstacle and unknown cells block reversing without rear lidar.
        # Forward-only stale routes behind the robot must not become reverse.
        robot_x = 0.
        pump(1.)
        for fill in (100, -1):
            cells = [0] * (600*400)
            for row in range(194, 206):
                for col in range(186, 190):  # x=-0.7 .. -0.5, behind body
                    cells[row*600+col] = fill
            grid.data = cells
            maps.publish(grid)
            pump(1.)
            command.path.poses = []
            for x in (0., -.1, -.5, -1., -3.):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.orientation.w = x, 1.
                command.path.poses.append(p)
            commands.clear()
            future = follow.send_goal_async(command)
            pump(1.)
            stopped = future.result().get_result_async()
            pump(.5)
            assert stopped.done() and stopped.result().status == 6
            assert not any(m.linear.x < -.01 for m in commands)
        grid.data = [0] * (600*400)
        maps.publish(grid)
        pump(1.)
        command.path.poses = []
        for x in (-3., -2.5, -2.):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.orientation.w = x, 1.
            command.path.poses.append(p)
        commands.clear()
        future = follow.send_goal_async(command)
        pump(1.)
        stopped = future.result().get_result_async()
        pump(.5)
        assert stopped.done() and stopped.result().status == 6
        assert not any(m.linear.x < -.01 for m in commands)
        future.result().cancel_goal_async()
        pump(.5)
        follow.destroy()

    finally:
        if handle is not None:
            handle.cancel_goal_async()
            pump(0.2)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(nav.pid, signal.SIGINT)
        nav.wait(timeout=20)
        buffer.terminate()
        buffer.wait(timeout=10)
        log.close()
