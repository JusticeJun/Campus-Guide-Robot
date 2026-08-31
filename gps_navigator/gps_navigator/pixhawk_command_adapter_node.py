import math

from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def bounded_motion(speed, yaw_rate, max_speed, minimum_turning_radius):
    speed = max(0.0, min(float(speed), max_speed))
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
        self.declare_parameter('maximum_speed_m_s', 0.6)
        self.declare_parameter('minimum_turning_radius_m', 1.7)
        self.last_command = None
        self.last_command_time_ns = None
        self.publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self.command_callback, 10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self.publish_command)

    def command_callback(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().error('Rejected non-finite Nav2 command')
            return
        self.last_command = bounded_motion(
            msg.linear.x, msg.angular.z,
            self.get_parameter('maximum_speed_m_s').value,
            self.get_parameter('minimum_turning_radius_m').value,
        )
        self.last_command_time_ns = self.get_clock().now().nanoseconds

    def publish_command(self):
        timeout_ns = self.get_parameter('command_timeout_s').value * 1e9
        stale = (
            self.last_command_time_ns is None
            or self.get_clock().now().nanoseconds
            - self.last_command_time_ns > timeout_ns
        )
        speed, yaw_rate = (0.0, 0.0) if stale else self.last_command
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
