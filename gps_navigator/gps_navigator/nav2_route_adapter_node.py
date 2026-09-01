import json
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
import yaml

from gps_navigator.geo_utils import gps_to_local_xy


def route_yaws(points):
    """Return a continuous forward orientation for each route point."""
    yaws = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            target = points[index + 1]
            yaws.append(math.atan2(target[1] - point[1], target[0] - point[0]))
        elif yaws:
            yaws.append(yaws[-1])
        else:
            yaws.append(0.0)
    return yaws


class Nav2RouteAdapter(Node):
    """Translate the campus graph route into a Nav2 navigation goal."""

    def __init__(self):
        super().__init__('nav2_route_adapter')
        default_file = (
            get_package_share_directory('gps_navigator')
            + '/config/waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('origin_latitude', 35.13484300)
        self.declare_parameter('origin_longitude', 129.1038000)
        self.behavior_tree = (
            get_package_share_directory('gps_navigator')
            + '/behavior_trees/navigate_through_poses_ackermann.xml'
        )
        with open(
            self.get_parameter('waypoints_file').value,
            'r', encoding='utf-8'
        ) as stream:
            self.waypoints = yaml.safe_load(stream)['waypoints']
        self.action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )
        self.lifecycle_client = self.create_client(
            GetState, '/bt_navigator/get_state'
        )
        self.pending_route = None
        self.nav2_active = False
        self.lifecycle_request = None
        self.waiting_logged = False
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/gps_navigation/route', self.route_callback, qos
        )
        self.create_timer(0.5, self.submit_pending_route)

    def route_callback(self, msg):
        try:
            route = json.loads(msg.data)['route']
            if not route or any(item not in self.waypoints for item in route):
                raise ValueError('empty route or unknown waypoint')
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid route message: {exc}')
            return
        self.pending_route = route

    def make_goal(self, route):
        origin_lat = self.get_parameter('origin_latitude').value
        origin_lon = self.get_parameter('origin_longitude').value
        points = [
            gps_to_local_xy(
                self.waypoints[item]['latitude'],
                self.waypoints[item]['longitude'],
                origin_lat, origin_lon,
            )
            for item in route
        ]
        goal = NavigateThroughPoses.Goal()
        goal.behavior_tree = self.behavior_tree
        stamp = self.get_clock().now().to_msg()
        for point, yaw in zip(points, route_yaws(points)):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)
        return goal

    def submit_pending_route(self):
        if self.pending_route is None:
            return
        if not self.nav2_active:
            if not self.waiting_logged:
                self.get_logger().info(
                    'Waiting for Nav2 to become active before sending route'
                )
                self.waiting_logged = True
            if (
                self.lifecycle_request is None
                and self.lifecycle_client.service_is_ready()
            ):
                self.lifecycle_request = self.lifecycle_client.call_async(
                    GetState.Request()
                )
                self.lifecycle_request.add_done_callback(
                    self.lifecycle_state_callback
                )
            return
        if not self.action_client.server_is_ready():
            return
        route = self.pending_route
        self.pending_route = None
        self.get_logger().info(
            f'Sending Nav2 route: {" -> ".join(route)}'
        )
        future = self.action_client.send_goal_async(self.make_goal(route))
        future.add_done_callback(self.goal_response_callback)

    def lifecycle_state_callback(self, future):
        self.lifecycle_request = None
        try:
            state = future.result().current_state
        except Exception as exc:  # service failures are retried by the timer
            self.get_logger().warning(f'Failed to get Nav2 state: {exc}')
            return
        if state.id != State.PRIMARY_STATE_ACTIVE:
            return
        self.nav2_active = True
        self.submit_pending_route()

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the campus route')
            return
        self.get_logger().info('Nav2 accepted the campus route')
        handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info(
            f'Nav2 route finished with status {future.result().status}'
        )


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2RouteAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
