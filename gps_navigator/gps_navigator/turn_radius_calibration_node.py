import csv
import math
from pathlib import Path
import time

import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


EARTH_RADIUS_M = 6371000.0


def heading_delta_deg(current, previous):
    """Return the shortest signed compass-heading change."""
    return (current - previous + 180.0) % 360.0 - 180.0


def gps_to_local_xy(latitude, longitude, origin_latitude, origin_longitude):
    """Convert WGS84 coordinates to a local east/north approximation."""
    mean_latitude = math.radians((latitude + origin_latitude) / 2.0)
    x = (
        EARTH_RADIUS_M
        * math.radians(longitude - origin_longitude)
        * math.cos(mean_latitude)
    )
    y = EARTH_RADIUS_M * math.radians(latitude - origin_latitude)
    return x, y


def solve_three_by_three(matrix, vector):
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(
            range(column, 3),
            key=lambda row: abs(augmented[row][column]),
        )
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError('Circle fit is singular')
        augmented[column], augmented[pivot] = (
            augmented[pivot], augmented[column]
        )
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(
                    augmented[row], augmented[column]
                )
            ]
    return [augmented[row][3] for row in range(3)]


def fit_circle(points):
    """Fit x^2 + y^2 + D*x + E*y + F = 0 by least squares."""
    if len(points) < 3:
        raise ValueError('At least three samples are required for circle fit')

    sum_x2 = sum(x * x for x, _ in points)
    sum_y2 = sum(y * y for _, y in points)
    sum_xy = sum(x * y for x, y in points)
    sum_x = sum(x for x, _ in points)
    sum_y = sum(y for _, y in points)
    rhs = [
        -sum(x * (x * x + y * y) for x, y in points),
        -sum(y * (x * x + y * y) for x, y in points),
        -sum(x * x + y * y for x, y in points),
    ]
    d_value, e_value, f_value = solve_three_by_three(
        [
            [sum_x2, sum_xy, sum_x],
            [sum_xy, sum_y2, sum_y],
            [sum_x, sum_y, float(len(points))],
        ],
        rhs,
    )
    center_x = -d_value / 2.0
    center_y = -e_value / 2.0
    radius_squared = center_x ** 2 + center_y ** 2 - f_value
    if radius_squared <= 0.0 or not math.isfinite(radius_squared):
        raise ValueError('Circle fit produced an invalid radius')
    return center_x, center_y, math.sqrt(radius_squared)


class TurnRadiusCalibration(Node):

    def __init__(self):
        super().__init__('turn_radius_calibration')
        self.declare_parameter('direction', 'LEFT')
        self.declare_parameter('forward_speed_m_s', 0.20)
        self.declare_parameter('max_yaw_rate_rad_s', 1.50)
        self.declare_parameter('target_turn_deg', 360.0)
        self.declare_parameter('startup_timeout_s', 20.0)
        self.declare_parameter('safety_timeout_s', 120.0)
        self.declare_parameter('gps_timeout_s', 2.0)
        self.declare_parameter('heading_timeout_s', 1.0)
        self.declare_parameter('max_heading_step_deg', 45.0)
        self.declare_parameter('stop_publish_duration_s', 1.0)
        self.declare_parameter('output_csv', '')

        self.direction = str(
            self.get_parameter('direction').value
        ).strip().upper()
        if self.direction not in ('LEFT', 'RIGHT'):
            raise ValueError('direction must be LEFT or RIGHT')
        for name in (
            'forward_speed_m_s',
            'max_yaw_rate_rad_s',
            'target_turn_deg',
            'startup_timeout_s',
            'safety_timeout_s',
            'gps_timeout_s',
            'heading_timeout_s',
            'max_heading_step_deg',
            'stop_publish_duration_s',
        ):
            if self.get_parameter(name).value <= 0.0:
                raise ValueError(f'{name} must be positive')

        self.gps = None
        self.heading = None
        self.last_gps_ns = None
        self.last_heading_ns = None
        self.previous_heading = None
        self.accumulated_turn_deg = 0.0
        self.samples = []
        self.started_ns = None
        self.node_started_ns = self.get_clock().now().nanoseconds
        self.finishing = False
        self.finish_started_ns = None
        self.result_reported = False
        self.exit_requested = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos,
        )
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos,
        )
        self.command_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )
        self.create_timer(0.1, self.control_loop)
        self.get_logger().warning(
            'Turn-radius calibration ready: direction=%s. Keep the area '
            'clear and do not run waypoint_follower at the same time.'
            % self.direction
        )

    @staticmethod
    def valid_gps(msg):
        return (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

    def gps_callback(self, msg):
        if not self.valid_gps(msg):
            self.gps = None
            self.last_gps_ns = None
            if self.started_ns is not None:
                self.finish('Invalid GPS fix', successful=False)
            return
        self.gps = (msg.latitude, msg.longitude)
        self.last_gps_ns = self.get_clock().now().nanoseconds
        if self.started_ns is not None and not self.finishing:
            self.samples.append(self.gps)

    def heading_callback(self, msg):
        if not math.isfinite(msg.data):
            self.heading = None
            self.last_heading_ns = None
            if self.started_ns is not None:
                self.finish('Invalid compass heading', successful=False)
            return
        self.heading = msg.data % 360.0
        self.last_heading_ns = self.get_clock().now().nanoseconds
        if self.started_ns is None or self.finishing:
            return
        if self.previous_heading is not None:
            delta = heading_delta_deg(self.heading, self.previous_heading)
            max_step = self.get_parameter('max_heading_step_deg').value
            if abs(delta) > max_step:
                self.finish(
                    f'Compass jump detected ({delta:.1f} deg)',
                    successful=False,
                )
                return
            expected_delta = -delta if self.direction == 'LEFT' else delta
            self.accumulated_turn_deg += expected_delta
        self.previous_heading = self.heading

    def input_fresh(self, stamp_ns, timeout_s):
        if stamp_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return 0 <= age_ns <= timeout_s * 1e9

    def make_command(self, speed, yaw_rate):
        command = PositionTarget()
        command.header.stamp = self.get_clock().now().to_msg()
        command.coordinate_frame = PositionTarget.FRAME_BODY_NED
        command.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        command.velocity.x = speed
        command.yaw_rate = yaw_rate
        return command

    def publish_stop(self):
        self.command_publisher.publish(self.make_command(0.0, 0.0))

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.finishing:
            self.publish_stop()
            stop_duration_ns = (
                self.get_parameter('stop_publish_duration_s').value * 1e9
            )
            if now_ns - self.finish_started_ns >= stop_duration_ns:
                self.exit_requested = True
            return

        if self.count_publishers('/mavros/setpoint_raw/local') > 1:
            self.finish(
                'Another setpoint publisher is active; refusing to drive',
                successful=False,
            )
            return

        gps_fresh = self.input_fresh(
            self.last_gps_ns, self.get_parameter('gps_timeout_s').value
        )
        heading_fresh = self.input_fresh(
            self.last_heading_ns,
            self.get_parameter('heading_timeout_s').value,
        )
        if self.started_ns is None:
            self.publish_stop()
            if gps_fresh and heading_fresh:
                self.started_ns = now_ns
                self.previous_heading = self.heading
                self.samples = [self.gps]
                self.get_logger().warning(
                    'Calibration motion started: direction=%s, speed=%.2f '
                    'm/s, fixed_yaw_rate=%.2f rad/s'
                    % (
                        self.direction,
                        self.get_parameter('forward_speed_m_s').value,
                        self.command_yaw_rate(),
                    )
                )
            elif now_ns - self.node_started_ns >= (
                self.get_parameter('startup_timeout_s').value * 1e9
            ):
                self.finish(
                    'GPS/heading not ready before startup timeout',
                    successful=False,
                )
            return

        if not gps_fresh or not heading_fresh:
            missing = []
            if not gps_fresh:
                missing.append('GPS')
            if not heading_fresh:
                missing.append('heading')
            self.finish(
                f'{"/".join(missing)} watchdog timeout', successful=False
            )
            return
        if now_ns - self.started_ns >= (
            self.get_parameter('safety_timeout_s').value * 1e9
        ):
            self.finish(
                '360-degree turn not completed before safety timeout',
                successful=False,
            )
            return
        if self.accumulated_turn_deg >= self.get_parameter(
            'target_turn_deg'
        ).value:
            self.finish('Target accumulated turn reached', successful=True)
            return

        self.command_publisher.publish(
            self.make_command(
                self.get_parameter('forward_speed_m_s').value,
                self.command_yaw_rate(),
            )
        )

    def command_yaw_rate(self):
        magnitude = self.get_parameter('max_yaw_rate_rad_s').value
        return magnitude if self.direction == 'LEFT' else -magnitude

    def local_points(self):
        if not self.samples:
            return []
        origin_latitude, origin_longitude = self.samples[0]
        return [
            gps_to_local_xy(
                latitude,
                longitude,
                origin_latitude,
                origin_longitude,
            )
            for latitude, longitude in self.samples
        ]

    def write_csv(self, points):
        configured_path = str(self.get_parameter('output_csv').value).strip()
        path = Path(configured_path) if configured_path else Path(
            '/tmp/turn_radius_%s_%s.csv'
            % (self.direction.lower(), time.strftime('%Y%m%d_%H%M%S'))
        )
        with path.open('w', newline='', encoding='utf-8') as output:
            writer = csv.writer(output)
            writer.writerow(['sample', 'latitude', 'longitude', 'x_m', 'y_m'])
            for index, ((latitude, longitude), (x, y)) in enumerate(
                zip(self.samples, points)
            ):
                writer.writerow([index, latitude, longitude, x, y])
        return path

    def report_result(self, successful):
        if self.result_reported:
            return
        self.result_reported = True
        points = self.local_points()
        try:
            output_path = self.write_csv(points)
            csv_result = str(output_path)
        except OSError as error:
            csv_result = f'write_failed: {error}'
        radius_result = 'unavailable'
        diameter_result = 'unavailable'
        if successful:
            try:
                _, _, radius = fit_circle(points)
                radius_result = f'{radius:.3f}'
                diameter_result = f'{2.0 * radius:.3f}'
            except ValueError as error:
                successful = False
                self.get_logger().error(f'Circle fitting failed: {error}')
        log = self.get_logger().info if successful else self.get_logger().error
        log(
            'turn_radius_result: direction=%s, samples=%d, '
            'accumulated_heading_change_deg=%.1f, radius_m=%s, '
            'diameter_m=%s, trajectory_csv=%s'
            % (
                self.direction,
                len(points),
                self.accumulated_turn_deg,
                radius_result,
                diameter_result,
                csv_result,
            )
        )

    def finish(self, reason, successful):
        if self.finishing:
            return
        self.finishing = True
        self.finish_started_ns = self.get_clock().now().nanoseconds
        self.publish_stop()
        log = self.get_logger().info if successful else self.get_logger().error
        log(f'Stopping calibration: {reason}')
        self.report_result(successful)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = TurnRadiusCalibration()
    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish('Interrupted by user', successful=False)
        end_time = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
