import heapq
import json
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rover_navigation.navigation_profile import (
    covered_graph,
    load_profile,
    map_to_gps,
)
import yaml

from rover_navigation.geo_utils import (
    gps_to_local_xy,
    haversine_distance,
    valid_nav_sat_fix,
)


def nearest_edge_projection(waypoints, latitude, longitude):
    """Project a GPS position onto the nearest graph edge in metric ENU."""
    positions = {
        waypoint_id: gps_to_local_xy(
            waypoint['latitude'], waypoint['longitude'], latitude, longitude
        )
        for waypoint_id, waypoint in waypoints.items()
    }
    directed_edges = {
        (start, end) for start, waypoint in waypoints.items() for end in waypoint['neighbors']
    }
    segments = {}
    for start, end in directed_edges:
        key = tuple(sorted((start, end)))
        segments.setdefault(key, set()).add((start, end))

    best = None
    for (a, b), directions in segments.items():
        ax, ay = positions[a]
        bx, by = positions[b]
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        if length_squared <= 0.0:
            continue
        raw_fraction = -(ax * dx + ay * dy) / length_squared
        fraction = max(0.0, min(1.0, raw_fraction))
        projection_x = ax + fraction * dx
        projection_y = ay + fraction * dy
        distance = math.hypot(projection_x, projection_y)
        candidate = {
            'a': a,
            'b': b,
            'fraction_from_a': fraction,
            'raw_fraction_from_a': raw_fraction,
            'edge_length_m': math.sqrt(length_squared),
            'distance_to_edge_m': distance,
            'directions': directions,
        }
        if best is None or distance < best['distance_to_edge_m']:
            best = candidate
    return best


class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        default_yaml = get_package_share_directory('rover_navigation') + '/config/waypoints.yaml'
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('position_topic', '/mavros/global_position/global')
        self.declare_parameter('destination', '')
        self.declare_parameter('navigation_profile', '')
        self.declare_parameter('route_policy', 'direct')
        self.declare_parameter('max_start_edge_distance_m', 15.0)
        self.declare_parameter('start_node_snap_distance_m', 1.0)
        self.declare_parameter('start_edge_endpoint_margin_m', 3.0)
        self.waypoints = self.load_waypoints(self.get_parameter('waypoints_file').value)
        self.localization_ready = False
        self.readiness_time = None
        self.create_subscription(Bool, '/navigation/ready', self.ready_callback, 1)
        self.profile = None
        profile_path = self.get_parameter('navigation_profile').value
        if profile_path:
            self.profile = load_profile(profile_path, 'validation')
            self.waypoints = covered_graph(
                self.load_waypoints(self.profile['waypoints_file']), self.profile
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_latitude = None
        self.current_longitude = None
        self.last_destination = None

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(String, '/gps_navigation/route', route_qos)
        position_qos = QoSProfile(depth=10)
        position_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        if self.profile is None:
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
                    raise RuntimeError(f'{waypoint_id} is missing field: {field}')
            for neighbor in waypoint['neighbors']:
                if neighbor not in waypoints:
                    raise RuntimeError(f'{waypoint_id} has unknown neighbor: {neighbor}')
        return waypoints

    def ready_callback(self, msg):
        self.localization_ready = msg.data
        self.readiness_time = self.get_clock().now()

    def position_callback(self, msg):
        if self.profile:
            # GNSS remains available to diagnostics; never overwrite metric pose.
            return
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

    def route_from_projected_start(self, destination_id):
        nearest_id = self.nearest_waypoint()
        nearest = self.waypoints[nearest_id]
        nearest_distance = haversine_distance(
            self.current_latitude,
            self.current_longitude,
            nearest['latitude'],
            nearest['longitude'],
        )
        snap_distance = float(self.get_parameter('start_node_snap_distance_m').value)
        if snap_distance < 0.0:
            raise ValueError('start_node_snap_distance_m must be non-negative')
        if nearest_distance <= snap_distance:
            route, distance = self.dijkstra(nearest_id, destination_id)
            return route, distance, None

        projection = nearest_edge_projection(
            self.waypoints, self.current_latitude, self.current_longitude
        )
        max_distance = float(self.get_parameter('max_start_edge_distance_m').value)
        if max_distance < 0.0:
            raise ValueError('max_start_edge_distance_m must be non-negative')
        if projection is None or projection['distance_to_edge_m'] > max_distance:
            route, distance = self.dijkstra(nearest_id, destination_id)
            return route, distance, None

        a = projection['a']
        b = projection['b']
        fraction = projection['fraction_from_a']
        raw_fraction = projection['raw_fraction_from_a']
        edge_length = projection['edge_length_m']
        endpoint_margin = float(self.get_parameter('start_edge_endpoint_margin_m').value)
        if endpoint_margin < 0.0:
            raise ValueError('start_edge_endpoint_margin_m must be non-negative')

        distance_from_a = fraction * edge_length
        distance_from_b = (1.0 - fraction) * edge_length
        if raw_fraction <= 0.0 or distance_from_a <= endpoint_margin:
            start_id = a
            connection_cost = projection['distance_to_edge_m'] + distance_from_a
        elif raw_fraction >= 1.0 or distance_from_b <= endpoint_margin:
            start_id = b
            connection_cost = projection['distance_to_edge_m'] + distance_from_b
        else:
            candidates = []
            # A directed A -> B edge permits an interior temporary start to
            # join toward B. Its reciprocal independently permits A.
            if (a, b) in projection['directions']:
                _, graph_cost = self.dijkstra(b, destination_id)
                if math.isfinite(graph_cost):
                    candidates.append((distance_from_b + graph_cost, b))
            if (b, a) in projection['directions']:
                _, graph_cost = self.dijkstra(a, destination_id)
                if math.isfinite(graph_cost):
                    candidates.append((distance_from_a + graph_cost, a))
            if not candidates:
                return None, math.inf, projection
            _, start_id = min(candidates, key=lambda item: item[0])
            connection_cost = projection['distance_to_edge_m'] + (
                distance_from_a if start_id == a else distance_from_b
            )

        # Projection chooses only the first real graph node. From that node
        # onward, preserve the ordinary graph route without further skipping.
        route, graph_cost = self.dijkstra(start_id, destination_id)
        if route is None:
            return None, math.inf, projection
        projection['selected_start'] = start_id
        total_cost = connection_cost + graph_cost
        return route, total_cost, projection

    def plan_if_needed(self):
        if self.profile and (
            not self.localization_ready
            or self.readiness_time is None
            or not 0 <= (self.get_clock().now() - self.readiness_time).nanoseconds <= 5e8
        ):
            return
        if self.profile:
            try:
                pose = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                age = (self.get_clock().now() - Time.from_msg(pose.header.stamp)).nanoseconds / 1e9
                if not 0.0 <= age <= 0.5:
                    return
                self.update_current_position(
                    *map_to_gps(
                        pose.transform.translation.x,
                        pose.transform.translation.y,
                        self.profile['alignment'],
                    )
                )
                projection = nearest_edge_projection(
                    self.waypoints, self.current_latitude, self.current_longitude
                )
                if self.get_parameter('route_policy').value == 'graph' and (
                    projection is None
                    or projection['distance_to_edge_m']
                    > self.get_parameter('max_start_edge_distance_m').value
                ):
                    self.get_logger().warning(
                        'Graph policy: current pose is outside reviewed edge entry area; '
                        'use direct policy for an off-graph start',
                        throttle_duration_sec=5.0,
                    )
                    return  # No GPS-style nearest-node fallback outside coverage.
            except TransformException:
                return
        destination_text = self.get_parameter('destination').value.strip()
        if (
            not destination_text
            or self.current_latitude is None
            or destination_text == self.last_destination
        ):
            return
        destination_id = self.resolve_destination(destination_text)
        if destination_id is None:
            self.get_logger().error(f'Unknown destination ID or name: {destination_text}')
            self.last_destination = destination_text
            return

        if self.profile and self.get_parameter('route_policy').value == 'direct':
            route, projection = [destination_id], None
            target = self.waypoints[destination_id]
            distance_m = haversine_distance(
                self.current_latitude,
                self.current_longitude,
                target['latitude'],
                target['longitude'],
            )
        else:
            route, distance_m, projection = self.route_from_projected_start(destination_id)
        self.last_destination = destination_text
        if route is None:
            self.get_logger().error(f'No route from current position to {destination_id}')
            return

        entry_edge = None
        if self.profile and projection is not None:
            first = route[0]
            other = projection['b'] if first == projection['a'] else projection['a']
            if first in self.waypoints[other]['neighbors']:
                entry_edge = [other, first]
        message = String()
        message.data = json.dumps(
            {
                'destination': destination_id,
                'route': route,
                'distance_m': distance_m,
                'entry_edge': entry_edge,
                'policy': self.get_parameter('route_policy').value if self.profile else 'gps',
            },
            ensure_ascii=False,
        )
        self.route_pub.publish(message)
        if self.profile and self.get_parameter('route_policy').value == 'direct':
            self.get_logger().info('Direct policy: only final destination sent to Nav2')
        elif projection is None:
            self.get_logger().info('Used nearest-waypoint start (node snap or edge fallback)')
        else:
            self.get_logger().info(
                'Temporary start projected onto '
                f'{projection["a"]}-{projection["b"]} '
                f'({projection["distance_to_edge_m"]:.1f} m from edge); '
                f'first graph node={projection["selected_start"]}'
            )
        self.get_logger().info(f'Route: {" -> ".join(route)} ({distance_m:.1f} m)')


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
