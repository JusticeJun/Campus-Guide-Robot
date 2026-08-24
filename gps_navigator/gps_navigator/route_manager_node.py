import json
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Header, String
import yaml

from gps_navigator.geo_utils import haversine_distance, valid_nav_sat_fix


def state_qos():
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class RouteManager(Node):

    def __init__(self):
        super().__init__('route_manager')
        default_file = (
            get_package_share_directory('gps_navigator')
            + '/config/waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter(
            'position_topic', '/mavros/global_position/global'
        )
        self.declare_parameter('arrival_radius_m', 3.0)
        self.declare_parameter('heartbeat_period_s', 1.0)
        with open(
            self.get_parameter('waypoints_file').value,
            'r', encoding='utf-8'
        ) as yaml_file:
            self.waypoints = yaml.safe_load(yaml_file)['waypoints']
        self.route = []
        self.target_index = 0
        self.completed = False
        self.last_log_ns = 0

        self.create_subscription(
            String, '/gps_navigation/route',
            self.route_callback, state_qos()
        )
        position_qos = QoSProfile(depth=10)
        position_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix,
            self.get_parameter('position_topic').value,
            self.position_callback,
            position_qos,
        )
        self.next_pub = self.create_publisher(
            NavSatFix, '/gps_navigation/next_waypoint', state_qos()
        )
        self.complete_pub = self.create_publisher(
            Bool, '/gps_navigation/route_complete', state_qos()
        )
        self.status_pub = self.create_publisher(
            String, '/gps_navigation/route_status', state_qos()
        )
        self.heartbeat_pub = self.create_publisher(
            Header, '/gps_navigation/route_manager_heartbeat', 10
        )
        heartbeat_period = self.get_parameter('heartbeat_period_s').value
        if heartbeat_period <= 0.0:
            raise ValueError('heartbeat_period_s must be positive')
        self.create_timer(heartbeat_period, self.publish_heartbeat)

    def publish_heartbeat(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = ''
        self.heartbeat_pub.publish(msg)

    def route_callback(self, msg):
        try:
            route = json.loads(msg.data)['route']
            if not route or any(item not in self.waypoints for item in route):
                raise ValueError('empty route or unknown waypoint')
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid route message: {exc}')
            return
        self.route = route
        self.target_index = 0
        self.completed = False
        self.publish_complete(False)
        self.publish_next()
        self.publish_status(None)
        self.get_logger().info(f'Accepted route: {" -> ".join(route)}')

    def publish_next(self):
        if not self.route or self.completed:
            return
        waypoint_id = self.route[self.target_index]
        waypoint = self.waypoints[waypoint_id]
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.latitude = float(waypoint['latitude'])
        msg.longitude = float(waypoint['longitude'])
        msg.altitude = math.nan
        self.next_pub.publish(msg)

    def publish_complete(self, value):
        msg = Bool()
        msg.data = value
        self.complete_pub.publish(msg)

    def position_callback(self, msg):
        if valid_nav_sat_fix(msg):
            self.update_current_position(msg.latitude, msg.longitude)

    def update_current_position(self, latitude, longitude):
        if not self.route or self.completed:
            return
        target_id = self.route[self.target_index]
        target = self.waypoints[target_id]
        distance = haversine_distance(
            latitude, longitude,
            target['latitude'], target['longitude'],
        )
        if distance <= self.get_parameter('arrival_radius_m').value:
            self.get_logger().info(
                f'Arrived at {target_id} ({target["name"]})'
            )
            if self.target_index == len(self.route) - 1:
                self.completed = True
                self.publish_complete(True)
                self.publish_status(distance)
                self.get_logger().info('Final destination reached')
                return
            self.target_index += 1
            self.publish_next()
            target_id = self.route[self.target_index]
            target = self.waypoints[target_id]
            distance = haversine_distance(
                latitude, longitude,
                target['latitude'], target['longitude'],
            )

        self.publish_status(distance)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1_000_000_000:
            previous = (
                self.route[self.target_index - 1]
                if self.target_index > 0 else 'START'
            )
            self.get_logger().info(
                f'edge={previous}->{target_id}, '
                f'next={target_id} ({target["name"]}), '
                f'distance={distance:.1f} m, '
                f'remaining={self.route[self.target_index:]}'
            )
            self.last_log_ns = now_ns

    def publish_status(self, distance):
        next_id = None if self.completed else self.route[self.target_index]
        previous = (
            self.route[self.target_index - 1]
            if self.target_index > 0 else 'START'
        )
        msg = String()
        msg.data = json.dumps({
            'edge': None if next_id is None else [previous, next_id],
            'next_waypoint': next_id,
            'distance_to_next_m': distance,
            'remaining_route': (
                [] if self.completed else self.route[self.target_index:]
            ),
            'complete': self.completed,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
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
