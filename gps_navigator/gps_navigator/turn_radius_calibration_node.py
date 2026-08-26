import csv
import math
from pathlib import Path
import statistics
import time

import rclpy
from mavros_msgs.msg import ManualControl, RCOut, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import yaml


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


def update_vehicle_constraints(
    data, direction, radius, steering, actual_pwm, speed,
    planning_radius_margin_ratio=None,
):
    constraints = data.setdefault('vehicle_constraints', {})
    constraints.pop('minimum_safe_turn_radius_m', None)
    constraints.pop('maximum_curvature_1_per_m', None)
    side = direction.lower()
    constraints[f'minimum_turn_radius_{side}_m'] = round(radius, 3)
    constraints['planning_radius_margin_ratio'] = (
        None
        if planning_radius_margin_ratio is None
        else planning_radius_margin_ratio
    )
    constraints['physical_min_turn_radius_m'] = None
    constraints['planning_min_turn_radius_m'] = None
    constraints['maximum_physical_curvature_1_per_m'] = None
    constraints['maximum_planning_curvature_1_per_m'] = None
    calibration = data.setdefault('calibration', {})
    calibration[side] = {
        'steering_command_normalized': steering,
        'actual_steering_pwm': actual_pwm,
        'speed_command_normalized': speed,
    }
    left_radius = constraints.get('minimum_turn_radius_left_m')
    right_radius = constraints.get('minimum_turn_radius_right_m')
    if left_radius is not None and right_radius is not None:
        physical_radius = max(left_radius, right_radius)
        constraints['physical_min_turn_radius_m'] = round(
            physical_radius, 3
        )
        constraints['maximum_physical_curvature_1_per_m'] = round(
            1.0 / physical_radius, 6
        )
        if planning_radius_margin_ratio is not None:
            planning_radius = physical_radius * (
                1.0 + planning_radius_margin_ratio
            )
            constraints['planning_min_turn_radius_m'] = round(
                planning_radius, 3
            )
            constraints['maximum_planning_curvature_1_per_m'] = round(
                1.0 / planning_radius, 6
            )
    return data


class TurnRadiusCalibration(Node):

    def __init__(self):
        super().__init__('turn_radius_calibration')
        self.declare_parameter('direction', 'LEFT')
        self.declare_parameter('steering_command_normalized', 1.0)
        self.declare_parameter('steering_reversed', False)
        self.declare_parameter('throttle_command_normalized', 0.15)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('endpoint_confirmed', False)
        self.declare_parameter('dry_run_hold_s', 2.0)
        self.declare_parameter('target_turn_deg', 360.0)
        self.declare_parameter('startup_timeout_s', 20.0)
        self.declare_parameter('safety_timeout_s', 120.0)
        self.declare_parameter('gps_timeout_s', 2.0)
        self.declare_parameter('heading_timeout_s', 1.0)
        self.declare_parameter('state_timeout_s', 2.0)
        self.declare_parameter('rc_out_timeout_s', 2.0)
        self.declare_parameter('max_heading_step_deg', 45.0)
        self.declare_parameter('stop_publish_duration_s', 1.0)
        self.declare_parameter('output_csv', '')
        self.declare_parameter(
            'constraints_yaml', '/tmp/vehicle_turn_constraints.yaml'
        )
        self.declare_parameter('planning_radius_margin_ratio', -1.0)
        self.declare_parameter('steering_output_channel', 1)

        self.direction = str(
            self.get_parameter('direction').value
        ).strip().upper()
        if self.direction not in ('LEFT', 'RIGHT'):
            raise ValueError('direction must be LEFT or RIGHT')
        for name in (
            'steering_command_normalized',
            'dry_run_hold_s',
            'target_turn_deg',
            'startup_timeout_s',
            'safety_timeout_s',
            'gps_timeout_s',
            'heading_timeout_s',
            'state_timeout_s',
            'rc_out_timeout_s',
            'max_heading_step_deg',
            'stop_publish_duration_s',
        ):
            if self.get_parameter(name).value <= 0.0:
                raise ValueError(f'{name} must be positive')
        steering_command = self.get_parameter(
            'steering_command_normalized'
        ).value
        throttle_command = self.get_parameter(
            'throttle_command_normalized'
        ).value
        if steering_command > 1.0:
            raise ValueError(
                'steering_command_normalized must be in (0.0, 1.0]'
            )
        if not 0.0 < throttle_command <= 0.3:
            raise ValueError(
                'throttle_command_normalized must be in (0.0, 0.3]'
            )
        steering_channel = self.get_parameter(
            'steering_output_channel'
        ).value
        if not isinstance(steering_channel, int) or steering_channel < 1:
            raise ValueError('steering_output_channel must be a positive int')
        planning_margin = self.get_parameter(
            'planning_radius_margin_ratio'
        ).value
        if planning_margin < 0.0 and planning_margin != -1.0:
            raise ValueError(
                'planning_radius_margin_ratio must be non-negative or -1.0 '
                'for unset'
            )

        self.gps = None
        self.heading = None
        self.last_gps_ns = None
        self.last_heading_ns = None
        self.vehicle_state = None
        self.last_state_ns = None
        self.actual_steering_pwm = None
        self.last_rc_out_ns = None
        self.steering_pwm_samples = []
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
        self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        self.create_subscription(
            RCOut, '/mavros/rc/out', self.rc_out_callback, qos
        )
        self.command_publisher = self.create_publisher(
            ManualControl, '/mavros/manual_control/send', 10
        )
        self.create_timer(0.1, self.control_loop)
        self.get_logger().warning(
            'Physical turn-radius calibration ready: direction=%s, '
            'dry_run=%s. Select MANUAL mode and do not run '
            'waypoint_follower at the same time.'
            % (self.direction, self.get_parameter('dry_run').value)
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
            if (
                self.started_ns is not None
                and not self.get_parameter('dry_run').value
            ):
                self.finish('Invalid GPS fix', successful=False)
            return
        self.gps = (msg.latitude, msg.longitude)
        self.last_gps_ns = self.get_clock().now().nanoseconds
        if self.started_ns is not None and not self.finishing:
            self.samples.append(
                (msg.latitude, msg.longitude, self.actual_steering_pwm)
            )

    def heading_callback(self, msg):
        if not math.isfinite(msg.data):
            self.heading = None
            self.last_heading_ns = None
            if (
                self.started_ns is not None
                and not self.get_parameter('dry_run').value
            ):
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

    def state_callback(self, msg):
        self.vehicle_state = msg
        self.last_state_ns = self.get_clock().now().nanoseconds

    def rc_out_callback(self, msg):
        channel_index = (
            self.get_parameter('steering_output_channel').value - 1
        )
        if channel_index >= len(msg.channels):
            self.actual_steering_pwm = None
            return
        self.actual_steering_pwm = msg.channels[channel_index]
        self.last_rc_out_ns = self.get_clock().now().nanoseconds
        if self.started_ns is not None and not self.finishing:
            self.steering_pwm_samples.append(self.actual_steering_pwm)

    def input_fresh(self, stamp_ns, timeout_s):
        if stamp_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return 0 <= age_ns <= timeout_s * 1e9

    def make_command(self, steering, throttle):
        command = ManualControl()
        command.header.stamp = self.get_clock().now().to_msg()
        command.x = 0.0
        command.y = steering * 1000.0
        command.z = throttle * 1000.0
        command.r = 0.0
        return command

    def publish_stop(self):
        self.command_publisher.publish(self.make_command(0.0, 0.0))

    def steering_command(self):
        magnitude = self.get_parameter(
            'steering_command_normalized'
        ).value
        command = -magnitude if self.direction == 'LEFT' else magnitude
        if self.get_parameter('steering_reversed').value:
            command *= -1.0
        return command

    def throttle_command(self):
        if self.get_parameter('dry_run').value:
            return 0.0
        return self.get_parameter('throttle_command_normalized').value

    def vehicle_state_valid(self):
        if self.vehicle_state is None or not self.vehicle_state.connected:
            return False, 'MAVROS is not connected to the FCU'
        if self.vehicle_state.mode.upper() != 'MANUAL':
            return False, 'Rover must be in MANUAL mode'
        if self.get_parameter('dry_run').value:
            if self.vehicle_state.armed:
                return False, 'Dry-run requires the vehicle to be disarmed'
        elif not self.vehicle_state.armed:
            return False, 'Calibration drive requires the vehicle to be armed'
        elif not self.get_parameter('endpoint_confirmed').value:
            return False, (
                'Run the disarmed dry-run first, then set '
                'endpoint_confirmed:=true'
            )
        return True, ''

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

        if self.count_publishers('/mavros/manual_control/send') > 1:
            self.finish(
                'Another manual-control publisher is active; '
                'refusing to drive',
                successful=False,
            )
            return
        if self.count_publishers('/mavros/setpoint_raw/local') > 0:
            self.finish(
                'A navigation setpoint publisher is active; refusing to drive',
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
        state_fresh = self.input_fresh(
            self.last_state_ns,
            self.get_parameter('state_timeout_s').value,
        )
        rc_out_fresh = self.input_fresh(
            self.last_rc_out_ns,
            self.get_parameter('rc_out_timeout_s').value,
        )
        state_valid, state_error = self.vehicle_state_valid()
        if self.started_ns is None:
            self.publish_stop()
            sensor_ready = (
                True
                if self.get_parameter('dry_run').value
                else gps_fresh and heading_fresh
            )
            if sensor_ready and state_fresh and rc_out_fresh and state_valid:
                self.started_ns = now_ns
                self.previous_heading = self.heading
                self.samples = [] if self.gps is None else [
                    (
                        self.gps[0],
                        self.gps[1],
                        self.actual_steering_pwm,
                    )
                ]
                self.get_logger().warning(
                    'Calibration command started: direction=%s, '
                    'steering_normalized=%.2f, throttle_normalized=%.2f'
                    % (
                        self.direction,
                        self.steering_command(),
                        self.throttle_command(),
                    )
                )
            elif now_ns - self.node_started_ns >= (
                self.get_parameter('startup_timeout_s').value * 1e9
            ):
                reason = state_error if not state_valid else (
                    'Required MAVROS inputs not ready before startup timeout'
                )
                self.finish(
                    reason, successful=False,
                )
            return

        if not state_fresh or not state_valid:
            reason = (
                'MAVROS state watchdog timeout'
                if not state_fresh else state_error
            )
            self.finish(reason, successful=False)
            return
        if not rc_out_fresh:
            self.finish(
                'Steering RC output watchdog timeout', successful=False
            )
            return
        if self.get_parameter('dry_run').value:
            if now_ns - self.started_ns >= (
                self.get_parameter('dry_run_hold_s').value * 1e9
            ):
                self.finish(
                    'Dry-run endpoint hold completed', successful=True
                )
                return
            self.command_publisher.publish(
                self.make_command(self.steering_command(), 0.0)
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
                self.steering_command(),
                self.throttle_command(),
            )
        )

    def local_points(self):
        if not self.samples:
            return []
        origin_latitude, origin_longitude, _ = self.samples[0]
        return [
            gps_to_local_xy(
                latitude,
                longitude,
                origin_latitude,
                origin_longitude,
            )
            for latitude, longitude, _ in self.samples
        ]

    def write_csv(self, points):
        configured_path = str(self.get_parameter('output_csv').value).strip()
        path = Path(configured_path) if configured_path else Path(
            '/tmp/turn_radius_%s_%s.csv'
            % (self.direction.lower(), time.strftime('%Y%m%d_%H%M%S'))
        )
        with path.open('w', newline='', encoding='utf-8') as output:
            writer = csv.writer(output)
            writer.writerow([
                'sample',
                'latitude',
                'longitude',
                'x_m',
                'y_m',
                'actual_steering_pwm',
            ])
            for index, ((latitude, longitude, pwm), (x, y)) in enumerate(
                zip(self.samples, points)
            ):
                writer.writerow([index, latitude, longitude, x, y, pwm])
        return path

    def representative_steering_pwm(self):
        if not self.steering_pwm_samples:
            return None
        return int(round(statistics.median(self.steering_pwm_samples)))

    def write_constraints_yaml(self, radius, actual_pwm):
        configured_path = str(
            self.get_parameter('constraints_yaml').value
        ).strip()
        if not configured_path:
            return 'disabled'
        path = Path(configured_path)
        data = {}
        if path.exists():
            try:
                with path.open(encoding='utf-8') as source:
                    data = yaml.safe_load(source) or {}
            except (OSError, yaml.YAMLError):
                data = {}
        planning_margin = self.get_parameter(
            'planning_radius_margin_ratio'
        ).value
        update_vehicle_constraints(
            data,
            self.direction,
            radius,
            self.steering_command(),
            actual_pwm,
            self.throttle_command(),
            None if planning_margin == -1.0 else planning_margin,
        )
        with path.open('w', encoding='utf-8') as output:
            yaml.safe_dump(data, output, sort_keys=False)
        return str(path)

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
        constraints_result = 'not_written'
        actual_pwm = self.representative_steering_pwm()
        dry_run = self.get_parameter('dry_run').value
        if successful and not dry_run:
            try:
                _, _, radius = fit_circle(points)
                radius_result = f'{radius:.3f}'
                diameter_result = f'{2.0 * radius:.3f}'
                constraints_result = self.write_constraints_yaml(
                    radius, actual_pwm
                )
            except ValueError as error:
                successful = False
                self.get_logger().error(f'Circle fitting failed: {error}')
            except OSError as error:
                successful = False
                self.get_logger().error(
                    f'Constraints YAML write failed: {error}'
                )
        log = self.get_logger().info if successful else self.get_logger().error
        log(
            'turn_radius_result: direction=%s, '
            'steering_command_normalized=%.2f, actual_steering_pwm=%s, '
            'speed_command_normalized=%.2f, samples=%d, '
            'accumulated_heading_change_deg=%.1f, radius_m=%s, '
            'diameter_m=%s, trajectory_csv=%s, constraints_yaml=%s'
            % (
                self.direction,
                self.steering_command(),
                'unavailable' if actual_pwm is None else actual_pwm,
                self.throttle_command(),
                len(points),
                self.accumulated_turn_deg,
                radius_result,
                diameter_result,
                csv_result,
                constraints_result,
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
