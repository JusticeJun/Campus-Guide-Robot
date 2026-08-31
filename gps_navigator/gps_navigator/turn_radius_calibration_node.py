import csv
import math
from pathlib import Path
import statistics
import struct
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from mavros_msgs.msg import Mavlink, PositionTarget, RCOut, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import yaml


EARTH_RADIUS_M = 6371000.0
DIAGNOSTIC_FIELDS = [
    'monotonic_time_ns',
    'ros_time_ns',
    'message_time_ns',
    'source',
    'event',
    'publish_sequence',
    'requested_velocity_x_m_s',
    'requested_yaw_rate_rad_s',
    'actual_forward_speed_m_s',
    'fcu_target_velocity_x_m_s',
    'fcu_target_yaw_rate_rad_s',
    'imu_yaw_rate_rad_s',
    'compass_heading_deg',
    'heading_derived_yaw_rate_rad_s',
    'steering_servo_pwm',
    'throttle_servo_pwm',
    'pid_tuning_axis',
    'pid_tuning_desired',
    'pid_tuning_achieved',
    'pid_tuning_ff',
    'pid_tuning_p',
    'pid_tuning_i',
    'pid_tuning_d',
    'vehicle_mode',
    'armed',
    'setpoint_publisher_count',
]

PID_TUNING_MESSAGE_ID = 194


def make_body_ned_velocity_yaw_rate_command(speed, yaw_rate):
    command = PositionTarget()
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


def yaw_rate_for_direction(direction, magnitude):
    return magnitude if direction == 'LEFT' else -magnitude


def heading_delta_deg(current, previous):
    """Return the shortest signed compass-heading change."""
    return (current - previous + 180.0) % 360.0 - 180.0


def compass_delta_to_ros_yaw_rate(current, previous, elapsed_s):
    """Derive ROS CCW-positive yaw rate from clockwise compass heading."""
    if elapsed_s <= 0.0:
        return None
    return -math.radians(heading_delta_deg(current, previous)) / elapsed_s


def message_stamp_ns(msg):
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def decode_pid_tuning(msg):
    """Decode MAVLink PID_TUNING without a generated Python dialect."""
    if (
        msg.msgid != PID_TUNING_MESSAGE_ID
        or msg.framing_status != Mavlink.FRAMING_OK
        or msg.len < 25
    ):
        return None
    payload = b''.join(
        int(word).to_bytes(8, byteorder='little', signed=False)
        for word in msg.payload64
    )[:msg.len]
    if len(payload) < 25:
        return None
    desired, achieved, ff, p, i, d, axis = struct.unpack_from(
        '<6fB', payload
    )
    return {
        'axis': axis,
        'desired': desired,
        'achieved': achieved,
        'ff': ff,
        'p': p,
        'i': i,
        'd': d,
    }


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
    data, direction, radius, yaw_rate, actual_pwm, speed,
    actual_pwm_min=None, actual_pwm_max=None,
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
        'command_pipeline': 'guided_body_ned_velocity_yaw_rate',
        'requested_yaw_rate_rad_s': yaw_rate,
        'actual_steering_pwm': actual_pwm,
        'actual_steering_pwm_min': actual_pwm_min,
        'actual_steering_pwm_max': actual_pwm_max,
        'speed_command_m_s': speed,
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
        self.declare_parameter('test_mode', 'SATURATION')
        self.declare_parameter('yaw_rate_rad_s', 1.5)
        self.declare_parameter('forward_speed_m_s', 0.20)
        self.declare_parameter('saturation_test_duration_s', 3.0)
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
        self.declare_parameter('diagnostic_csv', '')
        self.declare_parameter(
            'constraints_yaml', '/tmp/vehicle_turn_constraints.yaml'
        )
        self.declare_parameter('planning_radius_margin_ratio', -1.0)
        self.declare_parameter('steering_output_channel', 1)
        self.declare_parameter('throttle_output_channel', 3)
        self.declare_parameter('steering_pwm_low_endpoint', 1100)
        self.declare_parameter('steering_pwm_high_endpoint', 1900)
        self.declare_parameter('steering_pwm_endpoint_tolerance', 20)

        self.direction = str(
            self.get_parameter('direction').value
        ).strip().upper()
        if self.direction not in ('LEFT', 'RIGHT'):
            raise ValueError('direction must be LEFT or RIGHT')
        self.test_mode = str(
            self.get_parameter('test_mode').value
        ).strip().upper()
        if self.test_mode not in ('SATURATION', 'CIRCLE'):
            raise ValueError('test_mode must be SATURATION or CIRCLE')
        for name in (
            'yaw_rate_rad_s',
            'forward_speed_m_s',
            'saturation_test_duration_s',
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
        for channel_name in (
            'steering_output_channel', 'throttle_output_channel'
        ):
            channel = self.get_parameter(channel_name).value
            if not isinstance(channel, int) or channel < 1:
                raise ValueError(f'{channel_name} must be a positive int')
        low_endpoint = self.get_parameter(
            'steering_pwm_low_endpoint'
        ).value
        high_endpoint = self.get_parameter(
            'steering_pwm_high_endpoint'
        ).value
        endpoint_tolerance = self.get_parameter(
            'steering_pwm_endpoint_tolerance'
        ).value
        if not isinstance(low_endpoint, int) or not isinstance(
            high_endpoint, int
        ):
            raise ValueError('steering PWM endpoints must be integers')
        if low_endpoint >= high_endpoint:
            raise ValueError(
                'steering_pwm_low_endpoint must be below the high endpoint'
            )
        if not isinstance(endpoint_tolerance, int) or endpoint_tolerance < 0:
            raise ValueError(
                'steering_pwm_endpoint_tolerance must be a non-negative int'
            )
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
        self.actual_throttle_pwm = None
        self.actual_forward_speed = None
        self.last_rc_out_ns = None
        self.fcu_target_velocity_x = None
        self.fcu_target_yaw_rate = None
        self.imu_yaw_rate = None
        self.heading_derived_yaw_rate = None
        self.derivative_heading = None
        self.derivative_heading_monotonic_ns = None
        self.requested_velocity_x = None
        self.requested_yaw_rate = None
        self.publish_sequence = 0
        self.pid_tuning = None
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
        self.trace_write_failed = False
        self.diagnostic_output = None
        self.diagnostic_writer = None
        self.diagnostic_path = self.open_diagnostic_trace()

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
        self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.velocity_body_callback,
            qos,
        )
        self.create_subscription(
            Mavlink, '/mavros/mavlink/from', self.mavlink_callback, qos
        )
        self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/target_local',
            self.target_local_callback,
            qos,
        )
        self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            self.local_setpoint_callback,
            qos,
        )
        self.command_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )
        self.create_timer(0.1, self.control_loop)
        self.get_logger().warning(
            'Production-pipeline turn-radius calibration ready: mode=%s, '
            'direction=%s, speed=%.2f m/s, yaw_rate=%.3f rad/s. '
            'Select GUIDED mode, arm the Rover, and do not run '
            'another vehicle-command publisher at the same time. '
            'diagnostic_csv=%s'
            % (
                self.test_mode,
                self.direction,
                self.forward_speed_command(),
                self.yaw_rate_command(),
                self.diagnostic_path,
            )
        )

    def open_diagnostic_trace(self):
        configured = str(
            self.get_parameter('diagnostic_csv').value
        ).strip()
        path = Path(configured) if configured else Path(
            '/tmp/turn_radius_trace_%s_%s.csv'
            % (self.direction.lower(), time.strftime('%Y%m%d_%H%M%S'))
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        self.diagnostic_output = path.open(
            'w', newline='', encoding='utf-8', buffering=1
        )
        self.diagnostic_writer = csv.DictWriter(
            self.diagnostic_output, fieldnames=DIAGNOSTIC_FIELDS
        )
        self.diagnostic_writer.writeheader()
        return path

    def record_trace(
        self,
        source,
        event,
        message_time=None,
        receipt_monotonic_ns=None,
        receipt_ros_time_ns=None,
        **overrides,
    ):
        if self.diagnostic_writer is None or self.trace_write_failed:
            return
        state = self.vehicle_state
        row = {
            'monotonic_time_ns': (
                time.monotonic_ns()
                if receipt_monotonic_ns is None
                else receipt_monotonic_ns
            ),
            'ros_time_ns': (
                self.get_clock().now().nanoseconds
                if receipt_ros_time_ns is None
                else receipt_ros_time_ns
            ),
            'message_time_ns': '' if message_time is None else message_time,
            'source': source,
            'event': event,
            'publish_sequence': self.publish_sequence,
            'requested_velocity_x_m_s': self.requested_velocity_x,
            'requested_yaw_rate_rad_s': self.requested_yaw_rate,
            'actual_forward_speed_m_s': self.actual_forward_speed,
            'fcu_target_velocity_x_m_s': self.fcu_target_velocity_x,
            'fcu_target_yaw_rate_rad_s': self.fcu_target_yaw_rate,
            'imu_yaw_rate_rad_s': self.imu_yaw_rate,
            'compass_heading_deg': self.heading,
            'heading_derived_yaw_rate_rad_s': (
                self.heading_derived_yaw_rate
            ),
            'steering_servo_pwm': self.actual_steering_pwm,
            'throttle_servo_pwm': self.actual_throttle_pwm,
            'pid_tuning_axis': (
                None if self.pid_tuning is None
                else self.pid_tuning['axis']
            ),
            'pid_tuning_desired': (
                None if self.pid_tuning is None
                else self.pid_tuning['desired']
            ),
            'pid_tuning_achieved': (
                None if self.pid_tuning is None
                else self.pid_tuning['achieved']
            ),
            'pid_tuning_ff': (
                None if self.pid_tuning is None else self.pid_tuning['ff']
            ),
            'pid_tuning_p': (
                None if self.pid_tuning is None else self.pid_tuning['p']
            ),
            'pid_tuning_i': (
                None if self.pid_tuning is None else self.pid_tuning['i']
            ),
            'pid_tuning_d': (
                None if self.pid_tuning is None else self.pid_tuning['d']
            ),
            'vehicle_mode': None if state is None else state.mode,
            'armed': None if state is None else state.armed,
            'setpoint_publisher_count': self.count_publishers(
                '/mavros/setpoint_raw/local'
            ),
        }
        row.update(overrides)
        try:
            self.diagnostic_writer.writerow({
                key: '' if value is None else value
                for key, value in row.items()
            })
        except OSError as error:
            self.trace_write_failed = True
            self.get_logger().error(
                f'Diagnostic trace write failed; stopping: {error}'
            )

    def close_diagnostic_trace(self):
        if self.diagnostic_output is not None:
            self.diagnostic_output.flush()
            self.diagnostic_output.close()
            self.diagnostic_output = None
            self.diagnostic_writer = None

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
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        self.record_trace(
            'gps', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )
        if not self.valid_gps(msg):
            self.gps = None
            self.last_gps_ns = None
            if self.started_ns is not None:
                self.finish('Invalid GPS fix', successful=False)
            return
        self.gps = (msg.latitude, msg.longitude)
        self.last_gps_ns = self.get_clock().now().nanoseconds
        if self.started_ns is not None and not self.finishing:
            self.samples.append(
                (msg.latitude, msg.longitude, self.actual_steering_pwm)
            )

    def heading_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        if not math.isfinite(msg.data):
            self.heading = None
            self.last_heading_ns = None
            if self.started_ns is not None:
                self.finish('Invalid compass heading', successful=False)
            return
        self.heading = msg.data % 360.0
        if self.derivative_heading is not None:
            elapsed_s = (
                receipt_monotonic_ns - self.derivative_heading_monotonic_ns
            ) / 1e9
            self.heading_derived_yaw_rate = compass_delta_to_ros_yaw_rate(
                self.heading, self.derivative_heading, elapsed_s
            )
        self.derivative_heading = self.heading
        self.derivative_heading_monotonic_ns = receipt_monotonic_ns
        self.last_heading_ns = receipt_ros_time_ns
        self.record_trace(
            'compass_heading', 'received',
            receipt_monotonic_ns=receipt_monotonic_ns,
            receipt_ros_time_ns=receipt_ros_time_ns,
        )
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
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        self.vehicle_state = msg
        self.last_state_ns = receipt_ros_time_ns
        self.record_trace(
            'state', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def rc_out_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        steering_index = (
            self.get_parameter('steering_output_channel').value - 1
        )
        throttle_index = (
            self.get_parameter('throttle_output_channel').value - 1
        )
        if steering_index >= len(msg.channels):
            self.actual_steering_pwm = None
        else:
            self.actual_steering_pwm = msg.channels[steering_index]
            self.last_rc_out_ns = receipt_ros_time_ns
            if self.started_ns is not None and not self.finishing:
                self.steering_pwm_samples.append(self.actual_steering_pwm)
        self.actual_throttle_pwm = (
            None if throttle_index >= len(msg.channels)
            else msg.channels[throttle_index]
        )
        self.record_trace(
            'rc_out', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def imu_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        yaw_rate = msg.angular_velocity.z
        self.imu_yaw_rate = yaw_rate if math.isfinite(yaw_rate) else None
        self.record_trace(
            'imu', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def velocity_body_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        speed = msg.twist.linear.x
        self.actual_forward_speed = speed if math.isfinite(speed) else None
        self.record_trace(
            'velocity_body', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def mavlink_callback(self, msg):
        tuning = decode_pid_tuning(msg)
        if tuning is None:
            return
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        self.pid_tuning = tuning
        self.record_trace(
            'pid_tuning', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def target_local_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        self.fcu_target_velocity_x = msg.velocity.x
        self.fcu_target_yaw_rate = msg.yaw_rate
        self.record_trace(
            'target_local', 'received', message_stamp_ns(msg),
            receipt_monotonic_ns, receipt_ros_time_ns
        )

    def local_setpoint_callback(self, msg):
        receipt_monotonic_ns = time.monotonic_ns()
        receipt_ros_time_ns = self.get_clock().now().nanoseconds
        self.record_trace(
            'local_setpoint',
            'received',
            message_stamp_ns(msg),
            receipt_monotonic_ns,
            receipt_ros_time_ns,
            requested_velocity_x_m_s=msg.velocity.x,
            requested_yaw_rate_rad_s=msg.yaw_rate,
        )

    def input_fresh(self, stamp_ns, timeout_s):
        if stamp_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return 0 <= age_ns <= timeout_s * 1e9

    def make_command(self, speed, yaw_rate):
        command = make_body_ned_velocity_yaw_rate_command(speed, yaw_rate)
        command.header.stamp = self.get_clock().now().to_msg()
        return command

    def publish_command(self, command, event):
        self.publish_sequence += 1
        self.requested_velocity_x = command.velocity.x
        self.requested_yaw_rate = command.yaw_rate
        self.command_publisher.publish(command)
        self.record_trace(
            'command_publish', event, message_stamp_ns(command)
        )

    def publish_stop(self, event='stop_command'):
        self.publish_command(self.make_command(0.0, 0.0), event)

    def yaw_rate_command(self):
        magnitude = self.get_parameter('yaw_rate_rad_s').value
        return yaw_rate_for_direction(self.direction, magnitude)

    def forward_speed_command(self):
        return self.get_parameter('forward_speed_m_s').value

    def vehicle_state_valid(self):
        if self.vehicle_state is None or not self.vehicle_state.connected:
            return False, 'MAVROS is not connected to the FCU'
        if self.vehicle_state.mode.upper() != 'GUIDED':
            return False, 'Rover must be in GUIDED mode'
        if not self.vehicle_state.armed:
            return False, 'Calibration drive requires the vehicle to be armed'
        return True, ''

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.finishing:
            self.publish_stop('finishing_stop')
            stop_duration_ns = (
                self.get_parameter('stop_publish_duration_s').value * 1e9
            )
            if now_ns - self.finish_started_ns >= stop_duration_ns:
                self.exit_requested = True
            return

        if self.trace_write_failed:
            self.finish(
                'Diagnostic trace is unavailable', successful=False
            )
            return

        if self.count_publishers('/mavros/setpoint_raw/local') > 1:
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
            self.publish_stop('startup_stop')
            sensor_ready = gps_fresh and heading_fresh
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
                    'test_mode=%s, requested_yaw_rate_rad_s=%.3f, '
                    'speed_command_m_s=%.2f'
                    % (
                        self.direction,
                        self.test_mode,
                        self.yaw_rate_command(),
                        self.forward_speed_command(),
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
        if self.test_mode == 'SATURATION' and now_ns - self.started_ns >= (
            self.get_parameter('saturation_test_duration_s').value * 1e9
        ):
            self.finish(
                'Steering saturation test duration reached',
                successful=True,
            )
            return
        if self.test_mode == 'CIRCLE' and now_ns - self.started_ns >= (
            self.get_parameter('safety_timeout_s').value * 1e9
        ):
            self.finish(
                '360-degree turn not completed before safety timeout',
                successful=False,
            )
            return
        if (
            self.test_mode == 'CIRCLE'
            and self.accumulated_turn_deg
            >= self.get_parameter('target_turn_deg').value
        ):
            self.finish('Target accumulated turn reached', successful=True)
            return

        self.publish_command(
            self.make_command(
                self.forward_speed_command(),
                self.yaw_rate_command(),
            ),
            'drive_command',
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
                'requested_yaw_rate_rad_s',
                'speed_command_m_s',
            ])
            for index, ((latitude, longitude, pwm), (x, y)) in enumerate(
                zip(self.samples, points)
            ):
                writer.writerow([
                    index,
                    latitude,
                    longitude,
                    x,
                    y,
                    pwm,
                    self.yaw_rate_command(),
                    self.forward_speed_command(),
                ])
        return path

    def representative_steering_pwm(self):
        if not self.steering_pwm_samples:
            return None
        return int(round(statistics.median(self.steering_pwm_samples)))

    def steering_pwm_range(self):
        if not self.steering_pwm_samples:
            return None, None
        return min(self.steering_pwm_samples), max(self.steering_pwm_samples)

    def steering_endpoint_reached(self):
        pwm_min, pwm_max = self.steering_pwm_range()
        if pwm_min is None:
            return False
        tolerance = self.get_parameter(
            'steering_pwm_endpoint_tolerance'
        ).value
        return (
            pwm_min <= self.get_parameter(
                'steering_pwm_low_endpoint'
            ).value + tolerance
            or pwm_max >= self.get_parameter(
                'steering_pwm_high_endpoint'
            ).value - tolerance
        )

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
        actual_pwm_min, actual_pwm_max = self.steering_pwm_range()
        update_vehicle_constraints(
            data,
            self.direction,
            radius,
            self.yaw_rate_command(),
            actual_pwm,
            self.forward_speed_command(),
            actual_pwm_min,
            actual_pwm_max,
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
        if successful and self.test_mode == 'CIRCLE':
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
        actual_pwm_min, actual_pwm_max = self.steering_pwm_range()
        log(
            'turn_radius_result: test_mode=%s, direction=%s, '
            'requested_yaw_rate_rad_s=%.3f, actual_steering_pwm=%s, '
            'actual_steering_pwm_min=%s, actual_steering_pwm_max=%s, '
            'steering_endpoint_reached=%s, speed_command_m_s=%.2f, '
            'samples=%d, '
            'accumulated_heading_change_deg=%.1f, radius_m=%s, '
            'diameter_m=%s, trajectory_csv=%s, diagnostic_csv=%s, '
            'constraints_yaml=%s'
            % (
                self.test_mode,
                self.direction,
                self.yaw_rate_command(),
                'unavailable' if actual_pwm is None else actual_pwm,
                'unavailable' if actual_pwm_min is None else actual_pwm_min,
                'unavailable' if actual_pwm_max is None else actual_pwm_max,
                self.steering_endpoint_reached(),
                self.forward_speed_command(),
                len(points),
                self.accumulated_turn_deg,
                radius_result,
                diameter_result,
                csv_result,
                self.diagnostic_path,
                constraints_result,
            )
        )

    def finish(self, reason, successful):
        if self.finishing:
            return
        self.finishing = True
        self.finish_started_ns = self.get_clock().now().nanoseconds
        self.record_trace('lifecycle', 'finish_requested')
        self.publish_stop('finish_stop')
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
            node.publish_stop('ctrl_c_stop')
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.publish_stop('final_stop')
            rclpy.spin_once(node, timeout_sec=0.1)
        node.close_diagnostic_trace()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
