"""
Validate independent measurements, then expose measured-health EKF odometry.

The OdomInfo-compatible output is explicitly fusion health, not an RTAB match.
It shares each output odometry timestamp for the existing ExternalNav boundary.
"""

from copy import deepcopy
import json
import math
import numpy as np

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from rover_navigation.fusion_health import (
    MeasurementHealth, seconds, valid_motion, lidar_velocity_valid,
)


class FusionInputs(Node):
    def __init__(self):
        super().__init__('fusion_inputs')
        self.timeout = self.declare_parameter('measurement_timeout_s', 1.5).value
        self.health = MeasurementHealth(self.timeout)
        self.pending = {s: ({}, {}) for s in ('visual', 'lidar')}
        self.recovering = {'visual': True, 'lidar': True}
        self.last_input = {}
        self.offsets = {}
        self.filtered = None
        self.visual_reference = None
        self.output = self.create_publisher(Odometry, '/odom', 10)
        self.info = self.create_publisher(OdomInfo, '/localization/odom_info', 10)
        self.status = self.create_publisher(
            String, '/localization/fusion_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.measurements = {}
        for source, namespace in (('visual', '/vo'), ('lidar', '/lio')):
            self.measurements[source] = self.create_publisher(
                Odometry, f'/localization/{source}_measurement', 10)
            self.create_subscription(
                Odometry, namespace + '/odom',
                lambda m, s=source: self.on_odom(s, m), qos_profile_sensor_data)
            self.create_subscription(
                OdomInfo, namespace + '/odom_info_lite',
                lambda m, s=source: self.on_info(s, m), qos_profile_sensor_data)
        self.gyro = self.create_publisher(Imu, '/localization/fcu_gyro', 20)
        self.create_subscription(Imu, '/mavros/imu/data_raw',
                                 self.on_gyro, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_filtered,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.check_health)
        self.last_status = None

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, source, msg):
        self.pending[source][0][seconds(msg.header.stamp)] = msg
        self.pair(source, seconds(msg.header.stamp))

    def on_info(self, source, msg):
        stamp = seconds(msg.header.stamp)
        if msg.lost:
            if self.health.update(source, stamp, False):
                self.recovering[source] = True
            self.check_health()
        self.pending[source][1][stamp] = msg
        self.pair(source, stamp)

    def pair(self, source, stamp):
        odoms, infos = self.pending[source]
        for cache in (odoms, infos):
            while len(cache) > 12:
                del cache[next(iter(cache))]
        if stamp not in odoms or stamp not in infos:
            return
        odom, info = odoms.pop(stamp), infos.pop(stamp)
        valid = not info.lost and valid_motion(odom, self.now_seconds(), self.timeout)
        rejected_velocity = False
        if source == 'lidar' and valid:
            reference = self.visual_reference
            visual_twist = None
            if (reference is not None and 'visual' in self.health.active(self.now_seconds())
                    and abs(stamp-seconds(reference.header.stamp)) < 0.8):
                visual_twist = reference.twist.twist
            rejected_velocity = not lidar_velocity_valid(odom.twist.twist, visual_twist)
        previous = self.last_input.get(source)
        if previous is not None and stamp <= previous:
            return
        self.last_input[source] = stamp
        if rejected_velocity:
            # An excluded ICP delta is not an explicit tracker reset. Preserve
            # the last accepted observation's ORIGINAL timestamp; repeated bad
            # samples expire naturally and never refresh measurement health.
            return
        if not valid:
            self.health.update(source, stamp, False)
            self.recovering[source] = True
            return
        if self.recovering[source] or (previous is not None and stamp - previous > self.timeout):
            # First/reinitialized odometry often has a reset-derived zero/jump
            # velocity. Wait for the next real delta, without resetting the EKF.
            self.recovering[source] = False
            self.align_source(source, odom)
            return
        if not self.health.update(source, stamp, True):
            return
        if source == 'visual':
            self.visual_reference = odom
        output = deepcopy(odom)
        # Each local trajectory has its own origin. Re-anchor only on start or
        # recovery, to the continuous EKF trajectory at the measurement time.
        # EKF uses visual pose and lidar body twist; never both correlated
        # pose and velocity from the same source.
        x, y, angle = self.offsets[source]
        c, s = math.cos(angle), math.sin(angle)
        p = odom.pose.pose.position
        output.header.frame_id = 'odom'
        output.pose.pose.position.x = x + c * p.x - s * p.y
        output.pose.pose.position.y = y + s * p.x + c * p.y
        output.pose.pose.position.z = 0.0
        yaw = angle + self.yaw(odom.pose.pose.orientation)
        output.pose.pose.orientation.x = output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(yaw / 2)
        output.pose.pose.orientation.w = math.cos(yaw / 2)
        rotation = np.eye(6)
        rotation[:2, :2] = [[c, -s], [s, c]]
        covariance = rotation @ np.array(odom.pose.covariance).reshape(6, 6) @ rotation.T
        # Local registration uncertainty floors, not claimed global accuracy.
        for i in (0, 1):
            covariance[i, i] = max(covariance[i, i], 0.05 ** 2)
        covariance[5, 5] = max(covariance[5, 5], 0.05 ** 2)
        output.pose.covariance = covariance.ravel().tolist()
        if source == 'lidar':
            # Twist is already expressed in child_frame_id=base_link; do not
            # rotate it with the independent odometry-origin alignment.
            twist_covariance = list(output.twist.covariance)
            for i in (0, 7, 35):
                twist_covariance[i] = max(twist_covariance[i], 0.05 ** 2)
            output.twist.covariance = twist_covariance
        self.measurements[source].publish(output)

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def align_source(self, source, odom):
        x = y = yaw = 0.0
        if self.filtered is not None:
            f = self.filtered
            yaw = self.yaw(f.pose.pose.orientation)
            dt = seconds(odom.header.stamp) - seconds(f.header.stamp)
            dt = max(-self.timeout, min(self.timeout, dt))
            v, w = f.twist.twist.linear, f.twist.twist.angular
            x = f.pose.pose.position.x + dt * (math.cos(yaw) * v.x - math.sin(yaw) * v.y)
            y = f.pose.pose.position.y + dt * (math.sin(yaw) * v.x + math.cos(yaw) * v.y)
            yaw += dt * w.z
        angle = yaw - self.yaw(odom.pose.pose.orientation)
        p = odom.pose.pose.position
        self.offsets[source] = (x - math.cos(angle) * p.x + math.sin(angle) * p.y,
                                y - math.sin(angle) * p.x - math.cos(angle) * p.y, angle)

    def on_gyro(self, msg):
        # MAVROS raw gyro is independent of FCU ExternalNav pose/velocity.
        # The board is mounted FLU-aligned; never fuse its compass/EKF yaw back.
        if msg.header.frame_id not in ('base_link', 'base_link_frd'):
            return
        if not - 0.1 <= self.now_seconds() - seconds(msg.header.stamp) <= 0.5:
            return
        if not math.isfinite(msg.angular_velocity.z) or msg.angular_velocity_covariance[0] < 0:
            return
        output = deepcopy(msg)
        if output.header.frame_id == 'base_link_frd':
            output.angular_velocity.y *= -1
            output.angular_velocity.z *= -1
        output.header.frame_id = 'base_link'
        output.orientation_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        output.angular_velocity_covariance = [0.0] * 9
        for i in (0, 4, 8):
            output.angular_velocity_covariance[i] = 0.03 ** 2
        self.gyro.publish(output)

    def on_filtered(self, msg):
        active = self.health.active(self.now_seconds())
        if not active or not valid_motion(msg, self.now_seconds(), 0.5):
            return
        if msg.header.frame_id != 'odom':
            return
        q = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01:
            return
        self.filtered = msg
        self.output.publish(msg)
        info = OdomInfo()
        info.header = msg.header
        info.lost = False
        self.info.publish(info)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'base_link'
        p = msg.pose.pose.position
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.tf.sendTransform(transform)

    def check_health(self):
        active = self.health.active(self.now_seconds())
        if not active:
            info = OdomInfo()
            info.header.stamp = self.get_clock().now().to_msg()
            info.header.frame_id = 'odom'
            info.lost = True
            self.info.publish(info)
        state = json.dumps({'valid_sources': active, 'prediction_only': not bool(active)})
        if state != self.last_status:
            self.status.publish(String(data=state))
            self.last_status = state


def main(args=None):
    rclpy.init(args=args)
    node = FusionInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
