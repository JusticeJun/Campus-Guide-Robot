import csv
import math
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import Mavlink, PositionTarget, RCOut, State
from mavros_msgs.srv import EndpointAdd, EndpointDel
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from gps_navigator.turn_radius_calibration_node import (
    decode_pid_tuning,
    make_body_ned_velocity_yaw_rate_command,
    message_stamp_ns,
)


DIAGNOSTIC_FIELDS = [
    'monotonic_time_ns',
    'ros_time_ns',
    'message_time_ns',
    'source',
    'event',
    'publish_sequence',
    'requested_speed_m_s',
    'actual_speed_m_s',
    'fcu_target_speed_m_s',
    'fcu_target_yaw_rate_rad_s',
    'pid_tuning_axis',
    'pid_tuning_desired',
    'pid_tuning_achieved',
    'pid_tuning_ff',
    'pid_tuning_p',
    'pid_tuning_i',
    'pid_tuning_d',
    'pid_tuning_throttle_demand',
    'throttle_pwm',
    'steering_pwm',
    'vehicle_mode',
    'armed',
    'setpoint_publisher_count',
]


def make_straight_speed_command(speed):
    return make_body_ned_velocity_yaw_rate_command(speed, 0.0)


def pid_throttle_demand(tuning):
    """Return Rover's normalized pre-constrain speed-controller demand."""
    if tuning is None:
        return None
    return sum(tuning[key] for key in ('ff', 'p', 'i', 'd'))


class SpeedCalibration(Node):

    def __init__(self):
        super().__init__('speed_calibration')
        self.declare_parameter('forward_speed_m_s', 0.40)
        self.declare_parameter('duration_s', 8.0)
        self.declare_parameter('startup_timeout_s', 20.0)
        self.declare_parameter('state_timeout_s', 2.0)
        self.declare_parameter('rc_out_timeout_s', 2.0)
        self.declare_parameter('velocity_timeout_s', 2.0)
        self.declare_parameter('stop_publish_duration_s', 1.0)
        self.declare_parameter('diagnostic_csv', '')
        self.declare_parameter('steering_output_channel', 1)
        self.declare_parameter('throttle_output_channel', 3)
        self.declare_parameter(
            'mavros_router_add_endpoint_service',
            '/mavros/mavros_router/add_endpoint',
        )
        self.declare_parameter(
            'mavros_router_del_endpoint_service',
            '/mavros/mavros_router/del_endpoint',
        )

        for name in (
            'forward_speed_m_s',
            'duration_s',
            'startup_timeout_s',
            'state_timeout_s',
            'rc_out_timeout_s',
            'velocity_timeout_s',
            'stop_publish_duration_s',
        ):
            if self.get_parameter(name).value <= 0.0:
                raise ValueError(f'{name} must be positive')
        for name in ('steering_output_channel', 'throttle_output_channel'):
            value = self.get_parameter(name).value
            if not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive int')

        self.vehicle_state = None
        self.last_state_ns = None
        self.last_rc_out_ns = None
        self.last_velocity_ns = None
        self.actual_speed = None
        self.fcu_target_speed = None
        self.fcu_target_yaw_rate = None
        self.steering_pwm = None
        self.throttle_pwm = None
        self.pid_tuning = None
        self.mavlink_endpoint_id = None
        self.mavlink_endpoint_pending = False
        self.mavlink_endpoint_prefix = (
            f'/speed_calibration_mavlink_{self.get_process_id()}'
        )
        self.requested_speed = None
        self.publish_sequence = 0
        self.started_ns = None
        self.node_started_ns = self.get_clock().now().nanoseconds
        self.finishing = False
        self.finish_started_ns = None
        self.exit_requested = False
        self.trace_write_failed = False
        self.diagnostic_output = None
        self.diagnostic_writer = None
        self.diagnostic_path = self.open_diagnostic_trace()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        self.create_subscription(
            RCOut, '/mavros/rc/out', self.rc_out_callback, qos
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.velocity_callback,
            qos,
        )
        self.create_subscription(
            Mavlink,
            f'{self.mavlink_endpoint_prefix}/mavlink_source',
            self.mavlink_callback,
            qos,
        )
        self.endpoint_add_client = self.create_client(
            EndpointAdd,
            self.get_parameter(
                'mavros_router_add_endpoint_service'
            ).value,
        )
        self.endpoint_del_client = self.create_client(
            EndpointDel,
            self.get_parameter(
                'mavros_router_del_endpoint_service'
            ).value,
        )
        self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/target_local',
            self.target_callback,
            qos,
        )
        self.command_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )
        self.create_timer(0.1, self.control_loop)
        self.get_logger().warning(
            'Straight speed calibration ready: speed=%.2f m/s, '
            'duration=%.1f s. '
            'Select GUIDED, arm the Rover, and ensure no other setpoint '
            'publisher is running. diagnostic_csv=%s'
            % (
                self.forward_speed_command(),
                self.get_parameter('duration_s').value,
                self.diagnostic_path,
            )
        )

    @staticmethod
    def get_process_id():
        # A timestamp is sufficient to avoid collisions with stale diagnostic
        # endpoints while keeping the topic human-readable in ros2 topic list.
        return time.monotonic_ns()

    def ensure_mavlink_endpoint(self):
        if (
            self.mavlink_endpoint_id is not None
            or self.mavlink_endpoint_pending
        ):
            return
        if not self.endpoint_add_client.service_is_ready():
            return
        request = EndpointAdd.Request()
        request.url = self.mavlink_endpoint_prefix
        request.type = EndpointAdd.Request.TYPE_UAS
        future = self.endpoint_add_client.call_async(request)
        self.mavlink_endpoint_pending = True
        future.add_done_callback(self.mavlink_endpoint_callback)

    def mavlink_endpoint_callback(self, future):
        self.mavlink_endpoint_pending = False
        try:
            response = future.result()
        except Exception as error:  # rclpy service transport failure
            self.get_logger().error(
                f'Could not add MAVLink diagnostic endpoint: {error}'
            )
            return
        if not response.successful:
            self.get_logger().error(
                'Could not add MAVLink diagnostic endpoint: '
                f'{response.reason}'
            )
            return
        self.mavlink_endpoint_id = response.id
        self.get_logger().info(
            'MAVLink PID_TUNING capture connected through router endpoint '
            f'{self.mavlink_endpoint_prefix} (id={response.id})'
        )

    def remove_mavlink_endpoint(self):
        if (
            self.mavlink_endpoint_id is None
            or not self.endpoint_del_client.service_is_ready()
        ):
            return None
        request = EndpointDel.Request()
        request.id = self.mavlink_endpoint_id
        request.type = EndpointDel.Request.TYPE_UAS
        future = self.endpoint_del_client.call_async(request)
        self.mavlink_endpoint_id = None
        return future

    def open_diagnostic_trace(self):
        configured = str(self.get_parameter('diagnostic_csv').value).strip()
        path = Path(configured) if configured else Path(
            '/tmp/speed_calibration_%s.csv'
            % time.strftime('%Y%m%d_%H%M%S')
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

    def record_trace(self, source, event, message_time=None):
        if self.diagnostic_writer is None or self.trace_write_failed:
            return
        state = self.vehicle_state
        tuning = self.pid_tuning
        row = {
            'monotonic_time_ns': time.monotonic_ns(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'message_time_ns': '' if message_time is None else message_time,
            'source': source,
            'event': event,
            'publish_sequence': self.publish_sequence,
            'requested_speed_m_s': self.requested_speed,
            'actual_speed_m_s': self.actual_speed,
            'fcu_target_speed_m_s': self.fcu_target_speed,
            'fcu_target_yaw_rate_rad_s': self.fcu_target_yaw_rate,
            'pid_tuning_axis': None if tuning is None else tuning['axis'],
            'pid_tuning_desired': (
                None if tuning is None else tuning['desired']
            ),
            'pid_tuning_achieved': (
                None if tuning is None else tuning['achieved']
            ),
            'pid_tuning_ff': None if tuning is None else tuning['ff'],
            'pid_tuning_p': None if tuning is None else tuning['p'],
            'pid_tuning_i': None if tuning is None else tuning['i'],
            'pid_tuning_d': None if tuning is None else tuning['d'],
            'pid_tuning_throttle_demand': pid_throttle_demand(tuning),
            'throttle_pwm': self.throttle_pwm,
            'steering_pwm': self.steering_pwm,
            'vehicle_mode': None if state is None else state.mode,
            'armed': None if state is None else state.armed,
            'setpoint_publisher_count': self.count_publishers(
                '/mavros/setpoint_raw/local'
            ),
        }
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

    def state_callback(self, msg):
        self.vehicle_state = msg
        self.last_state_ns = self.get_clock().now().nanoseconds
        self.record_trace('state', 'received', message_stamp_ns(msg))

    def rc_out_callback(self, msg):
        steering_index = (
            self.get_parameter('steering_output_channel').value - 1
        )
        throttle_index = (
            self.get_parameter('throttle_output_channel').value - 1
        )
        self.steering_pwm = (
            None if steering_index >= len(msg.channels)
            else msg.channels[steering_index]
        )
        self.throttle_pwm = (
            None if throttle_index >= len(msg.channels)
            else msg.channels[throttle_index]
        )
        if self.steering_pwm is not None and self.throttle_pwm is not None:
            self.last_rc_out_ns = self.get_clock().now().nanoseconds
        self.record_trace('rc_out', 'received', message_stamp_ns(msg))

    def velocity_callback(self, msg):
        speed = msg.twist.linear.x
        self.actual_speed = speed if math.isfinite(speed) else None
        self.last_velocity_ns = (
            self.get_clock().now().nanoseconds
            if self.actual_speed is not None else None
        )
        self.record_trace('velocity_body', 'received', message_stamp_ns(msg))

    def mavlink_callback(self, msg):
        tuning = decode_pid_tuning(msg)
        # Rover reports its throttle-speed controller using the historical
        # PID_TUNING_ACCZ axis value (4), despite desired/achieved being m/s.
        if tuning is not None and tuning['axis'] == 4:
            self.pid_tuning = tuning
            self.record_trace('pid_tuning', 'received', message_stamp_ns(msg))

    def target_callback(self, msg):
        self.fcu_target_speed = msg.velocity.x
        self.fcu_target_yaw_rate = msg.yaw_rate
        self.record_trace('target_local', 'received', message_stamp_ns(msg))

    def input_fresh(self, stamp_ns, timeout_parameter):
        if stamp_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return 0 <= age_ns <= self.get_parameter(timeout_parameter).value * 1e9

    def vehicle_state_valid(self):
        if self.vehicle_state is None or not self.vehicle_state.connected:
            return False, 'MAVROS is not connected to the FCU'
        if self.vehicle_state.mode.upper() != 'GUIDED':
            return False, 'Rover must be in GUIDED mode'
        if not self.vehicle_state.armed:
            return False, 'Speed calibration requires the vehicle to be armed'
        return True, ''

    def forward_speed_command(self):
        return self.get_parameter('forward_speed_m_s').value

    def make_command(self, speed):
        command = make_straight_speed_command(speed)
        command.header.stamp = self.get_clock().now().to_msg()
        return command

    def publish_command(self, command, event):
        self.publish_sequence += 1
        self.requested_speed = command.velocity.x
        self.command_publisher.publish(command)
        self.record_trace('command_publish', event, message_stamp_ns(command))

    def publish_stop(self, event='stop_command'):
        self.publish_command(self.make_command(0.0), event)

    def finish(self, reason, successful):
        if self.finishing:
            return
        self.finishing = True
        self.finish_started_ns = self.get_clock().now().nanoseconds
        self.record_trace('lifecycle', 'finish_requested')
        self.publish_stop('finish_stop')
        log = self.get_logger().info if successful else self.get_logger().error
        log(
            f'Stopping speed calibration: {reason}; '
            f'CSV={self.diagnostic_path}'
        )

    def control_loop(self):
        self.ensure_mavlink_endpoint()
        now_ns = self.get_clock().now().nanoseconds
        if self.finishing:
            self.publish_stop('finishing_stop')
            if now_ns - self.finish_started_ns >= (
                self.get_parameter('stop_publish_duration_s').value * 1e9
            ):
                self.exit_requested = True
            return
        if self.trace_write_failed:
            self.finish('Diagnostic trace is unavailable', successful=False)
            return
        if self.count_publishers('/mavros/setpoint_raw/local') > 1:
            self.finish(
                'Another navigation setpoint publisher is active',
                successful=False,
            )
            return

        state_fresh = self.input_fresh(self.last_state_ns, 'state_timeout_s')
        rc_fresh = self.input_fresh(self.last_rc_out_ns, 'rc_out_timeout_s')
        velocity_fresh = self.input_fresh(
            self.last_velocity_ns, 'velocity_timeout_s'
        )
        state_valid, state_error = self.vehicle_state_valid()
        if self.started_ns is None:
            self.publish_stop('startup_stop')
            if (
                state_fresh
                and rc_fresh
                and velocity_fresh
                and state_valid
                and self.mavlink_endpoint_id is not None
            ):
                self.started_ns = now_ns
                self.get_logger().warning(
                    'Speed calibration started: speed=%.2f m/s, yaw_rate=0.0'
                    % self.forward_speed_command()
                )
            elif now_ns - self.node_started_ns >= (
                self.get_parameter('startup_timeout_s').value * 1e9
            ):
                if not state_valid:
                    reason = state_error
                elif self.mavlink_endpoint_id is None:
                    reason = (
                        'MAVROS router raw MAVLink endpoint unavailable; '
                        'check mavros_router add_endpoint service name'
                    )
                else:
                    reason = (
                        'Required MAVROS inputs not ready before startup '
                        'timeout'
                    )
                self.finish(reason, successful=False)
            return

        if not state_fresh or not state_valid:
            self.finish(
                'MAVROS state watchdog timeout' if not state_fresh
                else state_error,
                successful=False,
            )
            return
        if not rc_fresh or not velocity_fresh:
            missing = []
            if not rc_fresh:
                missing.append('RC output')
            if not velocity_fresh:
                missing.append('body velocity')
            self.finish(
                f'{"/".join(missing)} watchdog timeout',
                successful=False,
            )
            return
        if now_ns - self.started_ns >= (
            self.get_parameter('duration_s').value * 1e9
        ):
            self.finish('Requested duration reached', successful=True)
            return
        self.publish_command(
            self.make_command(self.forward_speed_command()), 'drive_command'
        )

    def close_diagnostic_trace(self):
        if self.diagnostic_output is not None:
            self.diagnostic_output.flush()
            self.diagnostic_output.close()
            self.diagnostic_output = None
            self.diagnostic_writer = None

    def destroy_node(self):
        self.close_diagnostic_trace()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SpeedCalibration()
    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.finish('Ctrl-C received', successful=False)
            deadline = time.monotonic() + node.get_parameter(
                'stop_publish_duration_s'
            ).value
            while rclpy.ok() and time.monotonic() < deadline:
                node.publish_stop('ctrl_c_stop')
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.publish_stop('final_stop')
            endpoint_future = node.remove_mavlink_endpoint()
            if endpoint_future is not None:
                rclpy.spin_until_future_complete(
                    node, endpoint_future, timeout_sec=1.0
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
