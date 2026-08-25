import heapq
import json
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import yaml

from gps_navigator.geo_utils import haversine_distance, valid_nav_sat_fix


class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        default_yaml = (
            get_package_share_directory('gps_navigator')
            + '/config/waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter(
            'position_topic', '/mavros/global_position/global'
        )
        self.declare_parameter('destination', '')
        self.waypoints = self.load_waypoints(
            self.get_parameter('waypoints_file').value
        )
        self.current_latitude = None
        self.current_longitude = None
        self.last_destination = None

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(
            String, '/gps_navigation/route', route_qos
        )
        position_qos = QoSProfile(depth=10)
        position_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix,
            self.get_parameter('position_topic').value,
            self.position_callback,
            position_qos,
        )
        self.create_timer(1.0, self.plan_if_needed)

    def load_waypoints(self, filename):
        with open(filename, 'r', encoding='utf-8') as yaml_file:
            data = yaml.safe_load(yaml_file)
        waypoints = data.get('waypoints', {})
        if not waypoints:
            raise RuntimeError('waypoints.yaml contains no waypoints')
        for waypoint_id, waypoint in waypoints.items():
            for field in ('name', 'latitude', 'longitude', 'neighbors'):
                if field not in waypoint:
                    raise RuntimeError(
                        f'{waypoint_id} is missing field: {field}'
                    )
            for neighbor in waypoint['neighbors']:
                if neighbor not in waypoints:
                    raise RuntimeError(
                        f'{waypoint_id} has unknown neighbor: {neighbor}'
                    )
        return waypoints

    def position_callback(self, msg):
        if valid_nav_sat_fix(msg):
            self.update_current_position(msg.latitude, msg.longitude)
        else:
            self.current_latitude = None
            self.current_longitude = None

    def update_current_position(self, latitude, longitude):
        self.current_latitude = latitude
        self.current_longitude = longitude

    def resolve_destination(self, destination):
        if destination in self.waypoints:
            return destination
        matches = [
            waypoint_id
            for waypoint_id, waypoint in self.waypoints.items()
            if waypoint['name'] == destination
        ]
        return matches[0] if len(matches) == 1 else None

    def nearest_waypoint(self):
        return min(
            self.waypoints,
            key=lambda waypoint_id: haversine_distance(
                self.current_latitude,
                self.current_longitude,
                self.waypoints[waypoint_id]['latitude'],
                self.waypoints[waypoint_id]['longitude'],
            ),
        )

    def dijkstra(self, start, goal):
        distances = {item: math.inf for item in self.waypoints}
        previous = {}
        distances[start] = 0.0
        queue = [(0.0, start)]
        while queue:
            current_distance, current = heapq.heappop(queue)
            if current_distance > distances[current]:
                continue
            if current == goal:
                break
            current_data = self.waypoints[current]
            for neighbor in current_data['neighbors']:
                neighbor_data = self.waypoints[neighbor]
                edge_cost = haversine_distance(
                    current_data['latitude'],
                    current_data['longitude'],
                    neighbor_data['latitude'],
                    neighbor_data['longitude'],
                )
                candidate = current_distance + edge_cost
                if candidate < distances[neighbor]:
                    distances[neighbor] = candidate
                    previous[neighbor] = current
                    heapq.heappush(queue, (candidate, neighbor))
        if not math.isfinite(distances[goal]):
            return None, math.inf
        route = [goal]
        while route[-1] != start:
            route.append(previous[route[-1]])
        route.reverse()
        return route, distances[goal]

    def plan_if_needed(self):
        destination_text = self.get_parameter('destination').value.strip()
        if (
            not destination_text
            or self.current_latitude is None
            or destination_text == self.last_destination
        ):
            return
        destination_id = self.resolve_destination(destination_text)
        if destination_id is None:
            self.get_logger().error(
                f'Unknown destination ID or name: {destination_text}'
            )
            self.last_destination = destination_text
            return

        start_id = self.nearest_waypoint()
        route, distance_m = self.dijkstra(start_id, destination_id)
        self.last_destination = destination_text
        if route is None:
            self.get_logger().error(
                f'No route from {start_id} to {destination_id}'
            )
            return

        message = String()
        message.data = json.dumps({
            'destination': destination_id,
            'route': route,
            'distance_m': distance_m,
        }, ensure_ascii=False)
        self.route_pub.publish(message)
        self.get_logger().info(
            f'Route: {" -> ".join(route)} ({distance_m:.1f} m)'
        )


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = PathPlanner()
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
