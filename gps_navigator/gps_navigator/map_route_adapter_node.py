"""Constrain the chosen graph route as an area and send only a final Nav2 goal."""

import copy
import json
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
import yaml

from gps_navigator.navigation_profile import covered_graph, load_profile
from gps_navigator.route_area import route_points, travel_grid


class MapRouteAdapter(Node):
    """Serialize route changes, preflight planning and terminal navigation state."""

    def __init__(self):
        super().__init__('map_route_adapter')
        self.declare_parameter('navigation_profile', '')
        self.declare_parameter('route_policy', 'direct')
        self.policy = self.get_parameter('route_policy').value
        if self.policy not in ('direct', 'graph'):
            raise ValueError('route_policy must be direct or graph')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.profile = load_profile(self.get_parameter('navigation_profile').value, 'validation')
        with open(self.profile['waypoints_file'], encoding='utf-8') as stream:
            self.graph = covered_graph(yaml.safe_load(stream)['waypoints'], self.profile)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.area_pub = self.create_publisher(OccupancyGrid, '/navigation/route_map', qos)
        self.state_pub = self.create_publisher(String, '/navigation/route_status', qos)
        self.active_pub = self.create_publisher(Bool, '/navigation/route_active', 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(String, '/gps_navigation/route', self.route_callback, qos)
        self.create_subscription(Bool, '/navigation/ready', self.ready_callback, 1)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_invalid = False
        self.source = None
        self.pending = None
        self.goal = None
        self.goal_handle = None
        self.ready_time = None
        self.mask_time = None
        self.phase = 'waiting_route'
        self.cancel_requested = False
        self.create_timer(0.1, self.tick)
        self.report('waiting_route')

    def report(self, state):
        self.phase = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(state)

    def ready_callback(self, msg):
        self.ready_time = self.get_clock().now() if msg.data else None

    def map_callback(self, msg):
        if self.source is not None:
            if self.source.info != msg.info or self.source.data != msg.data:
                self.map_invalid = True
                self.ready_time = None
                self.pending = {'route': []}
                self.report('map_changed_restart_required')
            return
        self.source = msg
        self.block_area()

    def block_area(self):
        if self.source is not None:
            area = copy.deepcopy(self.source)
            area.header.stamp = self.get_clock().now().to_msg()
            area.data = [100] * len(area.data)
            self.area_pub.publish(area)

    def route_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data.get('route'), list):
                raise ValueError('route must be a list')
            if data.get('policy', self.policy) != self.policy:
                raise ValueError('route policy mismatch')
            if self.policy == 'direct' and len(data['route']) > 1:
                raise ValueError('direct policy accepts only one final destination')
            if data['route']:
                route_points(
                    data['route'], self.graph, self.profile['alignment'], data.get('entry_edge')
                )
            self.pending = data
        except (ValueError, TypeError, KeyError) as exc:
            self.pending = {'route': []}
            self.report(f'invalid_route: {exc}')

    def ready(self):
        return (
            not self.map_invalid
            and self.ready_time is not None
            and 0 <= (self.get_clock().now() - self.ready_time).nanoseconds <= 5e8
        )

    def tick(self):
        status = Bool()
        status.data = self.phase == 'navigating' and self.pending is None and self.ready()
        self.active_pub.publish(status)
        if self.goal_handle is not None:
            if (self.pending is not None or not self.ready()) and not self.cancel_requested:
                self.goal_handle.cancel_goal_async()
                self.cancel_requested = True
                self.report('canceling_route')
            return
        if self.phase in ('planning', 'sending_navigation'):
            return
        if self.source is None or not self.ready():
            return
        if self.pending is not None:
            data, self.pending = self.pending, None
            if not data['route']:
                self.block_area()
                self.report('route_cleared')
                return
            points = route_points(
                data['route'], self.graph, self.profile['alignment'], data.get('entry_edge')
            )
            area = copy.deepcopy(self.source)
            if self.policy == 'graph':
                area.data = travel_grid(
                    area.info, area.data, points, self.profile['route_half_width_m']
                )
            else:
                try:
                    current = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                    points[0] = (current.transform.translation.x, current.transform.translation.y)
                except TransformException:
                    self.pending = data
                    return
            area.header.stamp = self.get_clock().now().to_msg()
            self.area_pub.publish(area)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = points[-1]
            yaw = math.atan2(points[-1][1] - points[-2][1], points[-1][0] - points[-2][0])
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.goal = pose
            self.mask_time = self.get_clock().now()
            self.report('waiting_costmap')
        if self.phase != 'waiting_costmap':
            return
        # StaticLayer updates at 5 Hz. Motion also requires a path validated
        # against this exact route map by the independent command guard.
        if (self.get_clock().now() - self.mask_time).nanoseconds < 1e9:
            return
        if not self.planner.server_is_ready() or not self.navigator.server_is_ready():
            return
        request = ComputePathToPose.Goal()
        request.goal = self.goal
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.use_start = False
        request.planner_id = 'GridBased'
        self.report('planning')
        self.planner.send_goal_async(request).add_done_callback(self.plan_response)

    def plan_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.report('planner_rejected')
                return
            handle.get_result_async().add_done_callback(self.plan_result)
        except Exception as exc:
            self.report(f'planner_failure: {exc}')

    def plan_result(self, future):
        try:
            result = future.result()
            if result.status != 4 or not result.result.path.poses:
                self.report('planner_failed')
                return
            if not self.ready() or self.pending is not None:
                self.report('planning_interrupted')
                return
            request = NavigateToPose.Goal()
            request.pose = self.goal
            request.pose.header.stamp = self.get_clock().now().to_msg()
            self.report('sending_navigation')
            self.navigator.send_goal_async(request).add_done_callback(self.nav_response)
        except Exception as exc:
            self.report(f'planner_failure: {exc}')

    def nav_response(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.goal_handle = None
                self.report('navigation_rejected')
                return
            self.cancel_requested = False
            self.goal_handle.get_result_async().add_done_callback(self.nav_result)
            self.report('navigating')
        except Exception as exc:
            self.goal_handle = None
            self.report(f'navigation_failure: {exc}')

    def nav_result(self, future):
        self.goal_handle = None
        self.cancel_requested = False
        try:
            status = future.result().status
            self.report(
                {4: 'completed', 5: 'canceled', 6: 'navigation_failed'}.get(
                    status, f'navigation_status_{status}'
                )
            )
        except Exception as exc:
            self.report(f'navigation_failure: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MapRouteAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
