import math

import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, Header


STOP_DISTANCE = 3.0


def state_qos():
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class Navigator(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self.lat = None
        self.lon = None
        self.heading = None
        self.target_lat = None
        self.target_lon = None
        self.last_gps_time_ns = None
        self.last_heading_time_ns = None
        self.last_heartbeat_time_ns = None
        self.route_complete = True
        self.watchdog_blocked = True
        self.last_warning_ns = {}
        self.last_diagnostic_ns = 0
        self.alignment_active = False

        self.declare_parameter('gps_timeout_s', 2.0)
        self.declare_parameter('heading_timeout_s', 2.0)
        self.declare_parameter('route_manager_timeout_s', 3.0)
        self.declare_parameter('warning_throttle_s', 5.0)
        self.declare_parameter('diagnostic_period_s', 1.0)
        self.declare_parameter('forward_speed_m_s', 0.3)
        self.declare_parameter('max_yaw_rate_rad_s', 1.5)
        self.declare_parameter('full_steering_error_deg', 30.0)
        self.declare_parameter('near_full_steering_error_deg', 15.0)
        self.declare_parameter('near_waypoint_distance_m', 10.0)
        self.declare_parameter('heading_deadband_deg', 2.0)
        self.declare_parameter('alignment_min_error_deg', 10.0)
        self.declare_parameter('alignment_reentry_error_deg', 30.0)
        self.declare_parameter('alignment_speed_m_s', 0.2)
        for name in (
            'gps_timeout_s',
            'heading_timeout_s',
            'route_manager_timeout_s',
            'warning_throttle_s',
            'diagnostic_period_s',
            'forward_speed_m_s',
            'max_yaw_rate_rad_s',
            'full_steering_error_deg',
            'near_full_steering_error_deg',
            'near_waypoint_distance_m',
            'alignment_min_error_deg',
            'alignment_reentry_error_deg',
            'alignment_speed_m_s',
        ):
            if self.get_parameter(name).value <= 0.0:
                raise ValueError(f'{name} must be positive')
        deadband = self.get_parameter('heading_deadband_deg').value
        full_steering_error = self.get_parameter(
            'full_steering_error_deg'
        ).value
        if deadband < 0.0 or deadband >= full_steering_error:
            raise ValueError(
                'heading_deadband_deg must be non-negative and less than '
                'full_steering_error_deg'
            )
        near_full_steering_error = self.get_parameter(
            'near_full_steering_error_deg'
        ).value
        if not deadband < near_full_steering_error <= full_steering_error:
            raise ValueError(
                'near_full_steering_error_deg must be greater than the '
                'deadband and no greater than full_steering_error_deg'
            )
        alignment_min_error = self.get_parameter(
            'alignment_min_error_deg'
        ).value
        alignment_reentry_error = self.get_parameter(
            'alignment_reentry_error_deg'
        ).value
        if alignment_reentry_error <= alignment_min_error:
            raise ValueError(
                'alignment_reentry_error_deg must be greater than '
                'alignment_min_error_deg'
            )

        mavros_qos = QoSProfile(depth=10)
        mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, mavros_qos
        )
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            self.heading_callback, mavros_qos
        )
        self.create_subscription(
            NavSatFix, '/gps_navigation/next_waypoint',
            self.target_callback, state_qos()
        )
        self.create_subscription(
            Bool, '/gps_navigation/route_complete',
            self.route_complete_callback, state_qos()
        )
        self.create_subscription(
            Header, '/gps_navigation/route_manager_heartbeat',
            self.heartbeat_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )
        self.create_timer(0.2, self.control_loop)

    @staticmethod
    def valid_coordinates(latitude, longitude):
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    def gps_callback(self, msg):
        if (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and self.valid_coordinates(msg.latitude, msg.longitude)
        ):
            self.lat = msg.latitude
            self.lon = msg.longitude
            self.last_gps_time_ns = self.get_clock().now().nanoseconds
        else:
            self.lat = None
            self.lon = None
            self.last_gps_time_ns = None

    def heading_callback(self, msg):
        self.heading = msg.data if math.isfinite(msg.data) else None
        self.last_heading_time_ns = (
            self.get_clock().now().nanoseconds
            if self.heading is not None else None
        )

    def target_callback(self, msg):
        if self.valid_coordinates(msg.latitude, msg.longitude):
            changed = (
                self.target_lat != msg.latitude
                or self.target_lon != msg.longitude
            )
            self.target_lat = msg.latitude
            self.target_lon = msg.longitude
            if changed:
                self.alignment_active = True
                self.get_logger().info(
                    f'새 목표 좌표: '
                    f'({self.target_lat:.7f}, {self.target_lon:.7f})'
                )
        else:
            self.target_lat = None
            self.target_lon = None

    def route_complete_callback(self, msg):
        self.route_complete = msg.data

    def heartbeat_callback(self, _msg):
        self.last_heartbeat_time_ns = self.get_clock().now().nanoseconds

    def input_is_fresh(self, stamp_ns, timeout_parameter):
        if stamp_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        timeout_ns = self.get_parameter(timeout_parameter).value * 1e9
        return 0 <= age_ns <= timeout_ns

    def warn_throttled(self, key, message):
        now_ns = self.get_clock().now().nanoseconds
        throttle_ns = (
            self.get_parameter('warning_throttle_s').value * 1e9
        )
        previous_ns = self.last_warning_ns.get(key)
        if previous_ns is None or now_ns - previous_ns >= throttle_ns:
            self.get_logger().warning(message)
            self.last_warning_ns[key] = now_ns

    def watchdog_reasons(self):
        reasons = []
        if not self.input_is_fresh(
            self.last_gps_time_ns, 'gps_timeout_s'
        ):
            reasons.append(('gps', 'GPS input timeout; commanding stop'))
        if not self.input_is_fresh(
            self.last_heading_time_ns, 'heading_timeout_s'
        ):
            reasons.append(
                ('heading', 'Compass heading timeout; commanding stop')
            )
        if not self.input_is_fresh(
            self.last_heartbeat_time_ns, 'route_manager_timeout_s'
        ):
            reasons.append(
                (
                    'route_manager',
                    'Route manager heartbeat timeout; commanding stop',
                )
            )
        if self.target_lat is None or self.target_lon is None:
            reasons.append(
                ('target', 'Next waypoint not received; commanding stop')
            )
        return reasons

    def calc_distance(self):
        radius = 6371000
        lat1 = math.radians(self.lat)
        lat2 = math.radians(self.target_lat)
        dlat = math.radians(self.target_lat - self.lat)
        dlon = math.radians(self.target_lon - self.lon)
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1) * math.cos(lat2)
            * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return radius * c

    def calc_bearing(self):
        lat1 = math.radians(self.lat)
        lat2 = math.radians(self.target_lat)
        dlon = math.radians(self.target_lon - self.lon)
        y = math.sin(dlon) * math.cos(lat2)
        x = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        )
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    @staticmethod
    def signed_heading_error(target_heading, current_heading):
        return (target_heading - current_heading + 180.0) % 360.0 - 180.0

    @staticmethod
    def steering_yaw_rate(
        heading_error_deg, max_yaw_rate, full_steering_error_deg,
        deadband_deg,
    ):
        # Geographic headings increase clockwise, while ROS yaw increases
        # counter-clockwise. Convert the sign at this boundary.
        error_magnitude = abs(heading_error_deg)
        if error_magnitude <= deadband_deg:
            return 0.0
        steering_ratio = min(
            1.0,
            (error_magnitude - deadband_deg)
            / (full_steering_error_deg - deadband_deg),
        )
        return -math.copysign(max_yaw_rate * steering_ratio, heading_error_deg)

    @staticmethod
    def full_steering_error_for_distance(
        distance_m, near_distance_m, near_error_deg, far_error_deg,
    ):
        if distance_m <= near_distance_m:
            return near_error_deg
        far_distance_m = near_distance_m * 2.0
        if distance_m >= far_distance_m:
            return far_error_deg
        ratio = (distance_m - near_distance_m) / near_distance_m
        return near_error_deg + ratio * (far_error_deg - near_error_deg)

    def make_command(self, speed, yaw_rate):
        command = PositionTarget()
        command.header.stamp = self.get_clock().now().to_msg()
        command.coordinate_frame = PositionTarget.FRAME_BODY_NED
        command.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        command.velocity.x = speed
        command.yaw_rate = yaw_rate
        return command

    def publish_stop(self):
        self.cmd_pub.publish(self.make_command(0.0, 0.0))

    def log_diagnostics(
        self, distance, target_bearing, error, command, control_mode,
        full_steering_error,
    ):
        now_ns = self.get_clock().now().nanoseconds
        period_ns = (
            self.get_parameter('diagnostic_period_s').value * 1e9
        )
        if now_ns - self.last_diagnostic_ns < period_ns:
            return
        self.get_logger().info(
            'navigation_control: '
            f'heading_source=/mavros/global_position/compass_hdg, '
            f'heading_deg={self.heading:.1f}, '
            f'target=({self.target_lat:.7f},{self.target_lon:.7f}), '
            f'target_bearing_deg={target_bearing:.1f}, '
            f'heading_error_deg={error:.1f}, '
            f'steering_yaw_rate_rad_s={command.yaw_rate:.3f}, '
            f'control_mode={control_mode}, '
            f'full_steering_error_deg={full_steering_error:.1f}, '
            f'final_frame=BODY_NED, '
            f'final_velocity_x_m_s={command.velocity.x:.2f}, '
            f'distance_m={distance:.1f}'
        )
        self.last_diagnostic_ns = now_ns

    def control_loop(self):
        if self.route_complete:
            self.watchdog_blocked = True
            self.publish_stop()
            return

        reasons = self.watchdog_reasons()
        if reasons:
            for key, message in reasons:
                self.warn_throttled(key, message)
            self.watchdog_blocked = True
            self.publish_stop()
            return

        if self.watchdog_blocked:
            self.get_logger().info(
                'Watchdog inputs restored; resuming waypoint following'
            )
            self.watchdog_blocked = False

        distance = self.calc_distance()
        target_heading = self.calc_bearing()
        error = self.signed_heading_error(target_heading, self.heading)

        if distance <= STOP_DISTANCE:
            self.publish_stop()
            self.get_logger().info(f'도착! 거리:{distance:.1f}m 정지')
            return

        max_yaw_rate = self.get_parameter('max_yaw_rate_rad_s').value
        full_steering_error = self.full_steering_error_for_distance(
            distance,
            self.get_parameter('near_waypoint_distance_m').value,
            self.get_parameter('near_full_steering_error_deg').value,
            self.get_parameter('full_steering_error_deg').value,
        )
        steering = self.steering_yaw_rate(
            error,
            max_yaw_rate,
            full_steering_error,
            self.get_parameter('heading_deadband_deg').value,
        )
        alignment_min_error = self.get_parameter(
            'alignment_min_error_deg'
        ).value
        alignment_reentry_error = self.get_parameter(
            'alignment_reentry_error_deg'
        ).value
        if self.alignment_active and abs(error) <= alignment_min_error:
            self.alignment_active = False
            self.get_logger().info(
                'Heading aligned; switching to waypoint tracking speed'
            )
        elif (
            not self.alignment_active
            and abs(error) >= alignment_reentry_error
        ):
            self.alignment_active = True
            self.get_logger().warning(
                'Large heading error; reducing speed for realignment'
            )

        control_mode = 'ALIGNING' if self.alignment_active else 'TRACKING'
        speed_parameter = (
            'alignment_speed_m_s'
            if self.alignment_active else 'forward_speed_m_s'
        )
        command = self.make_command(
            self.get_parameter(speed_parameter).value, steering
        )
        self.cmd_pub.publish(command)
        self.log_diagnostics(
            distance, target_heading, error, command, control_mode,
            full_steering_error,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
