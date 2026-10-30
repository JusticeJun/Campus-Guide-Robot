import math
import time

from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


def bounded_motion(speed, yaw_rate, max_speed, minimum_turning_radius):
    speed = max(0.0, min(float(speed), max_speed, 0.4))
    if speed == 0.0:
        return 0.0, 0.0
    max_yaw_rate = speed / minimum_turning_radius
    return speed, max(-max_yaw_rate, min(float(yaw_rate), max_yaw_rate))


def make_position_target(speed, yaw_rate):
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
    command.velocity.y = 0.0
    command.yaw_rate = yaw_rate
    return command


class PixhawkCommandAdapter(Node):
    """Keep MAVROS/Pixhawk as the low-level boundary for Nav2 commands."""

    def __init__(self):
        super().__init__('pixhawk_command_adapter')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('maximum_speed_m_s', 0.4)
        self.declare_parameter('minimum_turning_radius_m', 1.7)
        self.hold_on_fault = self.declare_parameter('hold_on_fault', False).value
        self.hold_pending = self.guard_stop = False
        self.fcu_state = None
        self.fcu_state_time = -float('inf')
        self.hold_future = None
        self.hold_request_time = -float('inf')
        if self.hold_on_fault:
            self.hold_client = self.create_client(SetMode, '/mavros/set_mode')
            self.create_subscription(
                State, '/mavros/state', self.state_callback, qos_profile_sensor_data
            )
            self.create_subscription(
                Bool,
                '/navigation/stop_latched',
                self.stop_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        self.last_command = None
        self.last_command_time_ns = None
        self.publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.command_callback,
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self.publish_command)

    def command_callback(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.request_fault_hold('non-finite command')
            self.last_command = None
            self.last_command_time_ns = None
            self.publish_command()
            self.get_logger().error('Rejected non-finite Nav2 command')
            return
        self.last_command = bounded_motion(
            msg.linear.x,
            msg.angular.z,
            self.get_parameter('maximum_speed_m_s').value,
            self.get_parameter('minimum_turning_radius_m').value,
        )
        self.last_command_time_ns = self.get_clock().now().nanoseconds

    def state_callback(self, msg):
        self.fcu_state = msg
        self.fcu_state_time = time.monotonic()
        self.service_fault_hold()

    def stop_callback(self, msg):
        self.guard_stop = msg.data
        if msg.data:
            self.request_fault_hold('navigation stop latched')
        self.service_fault_hold()

    def request_fault_hold(self, reason):
        if not getattr(self, 'hold_on_fault', False):
            return
        if not self.hold_pending:
            self.get_logger().error('Requesting FCU HOLD: ' + reason)
        self.hold_pending = True

    def service_fault_hold(self):
        if not getattr(self, 'hold_on_fault', False) or not self.hold_pending:
            return
        state = self.fcu_state
        if state is None or not state.connected or time.monotonic() - self.fcu_state_time > 2.0:
            return
        if state.mode == 'HOLD':
            if self.hold_future is not None:
                self.hold_client.remove_pending_request(self.hold_future)
                self.hold_future = None
            # A service acknowledgement is not proof of a mode change. Require
            # FCU telemetry and a cleared guard before accepting a new command.
            if not self.guard_stop:
                self.hold_pending = False
                self.last_command = self.last_command_time_ns = None
            return
        if state.mode != 'GUIDED':
            return  # Never override an operator's MANUAL/other mode.
        now = time.monotonic()
        if self.hold_future is not None:
            if self.hold_future.done():
                try:
                    reply = self.hold_future.result()
                    if reply is None or not reply.mode_sent:
                        self.get_logger().error('FCU HOLD request rejected; will retry')
                except Exception as error:
                    self.get_logger().error(f'FCU HOLD request failed: {error}')
                self.hold_future = None
            elif now - self.hold_request_time < 2.0:
                return
            else:
                self.hold_client.remove_pending_request(self.hold_future)
                self.hold_future = None
        if now - self.hold_request_time < 1.0 or not self.hold_client.service_is_ready():
            return
        request = SetMode.Request()
        request.custom_mode = 'HOLD'
        self.hold_future = self.hold_client.call_async(request)
        self.hold_request_time = now

    def publish_command(self):
        timeout_ns = self.get_parameter('command_timeout_s').value * 1e9
        stale = (
            self.last_command_time_ns is None
            or not 0
            <= self.get_clock().now().nanoseconds - self.last_command_time_ns
            <= timeout_ns
        )
        if stale and self.last_command is not None and self.last_command[0] > 0.0:
            self.request_fault_hold('command watchdog expired during motion')
        self.service_fault_hold()
        blocked = stale or getattr(self, 'hold_pending', False)
        speed, yaw_rate = (0.0, 0.0) if blocked else (self.last_command or (0.0, 0.0))
        command = make_position_target(speed, yaw_rate)
        command.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(command)

    def destroy_node(self):
        for _ in range(3):
            command = make_position_target(0.0, 0.0)
            command.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(command)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PixhawkCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
