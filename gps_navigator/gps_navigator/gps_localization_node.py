import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from gps_navigator.geo_utils import (
    compass_heading_to_yaw,
    gps_to_local_xy,
    valid_nav_sat_fix,
)


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class GpsLocalization(Node):
    """Expose the current GPS-only pose behind standard ROS interfaces."""

    def __init__(self):
        super().__init__('gps_localization')
        self.declare_parameter(
            'position_topic', '/mavros/global_position/global'
        )
        self.declare_parameter(
            'heading_topic', '/mavros/global_position/compass_hdg'
        )
        self.declare_parameter('origin_latitude', 35.13484300)
        self.declare_parameter('origin_longitude', 129.1038000)
        self.declare_parameter('gps_timeout_s', 2.0)
        self.declare_parameter('heading_timeout_s', 2.0)
        self.declare_parameter('position_variance_m2', 4.0)
        self.declare_parameter('yaw_variance_rad2', 0.08)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.position = None
        self.yaw = None
        self.position_time_ns = None
        self.heading_time_ns = None
        self.dynamic_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix, self.get_parameter('position_topic').value,
            self.position_callback, qos,
        )
        self.create_subscription(
            Float64, self.get_parameter('heading_topic').value,
            self.heading_callback, qos,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.publish_map_to_odom()
        self.create_timer(1.0 / rate, self.publish_pose)

    def publish_map_to_odom(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

    def position_callback(self, msg):
        if not valid_nav_sat_fix(msg):
            self.position = None
            return
        self.position = gps_to_local_xy(
            msg.latitude, msg.longitude,
            self.get_parameter('origin_latitude').value,
            self.get_parameter('origin_longitude').value,
        )
        self.position_time_ns = self.get_clock().now().nanoseconds

    def heading_callback(self, msg):
        if not math.isfinite(msg.data):
            self.yaw = None
            return
        self.yaw = compass_heading_to_yaw(msg.data)
        self.heading_time_ns = self.get_clock().now().nanoseconds

    def fresh(self, stamp_ns, parameter):
        if stamp_ns is None:
            return False
        timeout = float(self.get_parameter(parameter).value)
        return self.get_clock().now().nanoseconds - stamp_ns <= timeout * 1e9

    def publish_pose(self):
        if (
            self.position is None or self.yaw is None
            or not self.fresh(self.position_time_ns, 'gps_timeout_s')
            or not self.fresh(self.heading_time_ns, 'heading_timeout_s')
        ):
            return
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        position_variance = self.get_parameter('position_variance_m2').value
        yaw_variance = self.get_parameter('yaw_variance_rad2').value
        odom.pose.covariance[0] = position_variance
        odom.pose.covariance[7] = position_variance
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = yaw_variance
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.dynamic_tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GpsLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
