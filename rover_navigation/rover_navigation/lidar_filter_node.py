"""Crop the front scan without inventing clear rays behind the rover."""

from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def crop_scan(msg, half_angle_deg=100.0, sensor_yaw=0.0):
    if msg.angle_increment <= 0 or not math.isfinite(msg.angle_increment):
        raise ValueError('expected ascending scan angles')
    if not 0 < half_angle_deg <= 180:
        raise ValueError('front half-angle must be in (0,180]')
    half = math.radians(half_angle_deg)
    if not math.isfinite(sensor_yaw):
        raise ValueError('sensor yaw must be finite')
    # Keep ray order/time even if the vehicle-front sector wraps in laser frame.
    selected = [i for i in range(len(msg.ranges))
                if abs(math.atan2(math.sin(msg.angle_min + i * msg.angle_increment + sensor_yaw),
                                  math.cos(msg.angle_min + i * msg.angle_increment + sensor_yaw)))
                <= half + 1e-6]
    first, last = (selected[0], selected[-1]) if selected else (1, 0)
    if first > last:
        raise ValueError('scan does not cover the configured front sector')
    out = deepcopy(msg)
    out.ranges = list(msg.ranges[first:last + 1])
    out.intensities = list(msg.intensities[first:last + 1]) if msg.intensities else []
    out.angle_min = msg.angle_min + first * msg.angle_increment
    out.angle_max = msg.angle_min + last * msg.angle_increment
    # REP-117: NaN marks rejected/self returns, not free-space evidence.
    for i, distance in enumerate(out.ranges):
        angle = out.angle_min + i * out.angle_increment + sensor_yaw
        if abs(math.atan2(math.sin(angle), math.cos(angle))) > half + 1e-6:
            out.ranges[i] = float('nan')
            continue
        if distance == -math.inf or (math.isfinite(distance) and distance > msg.range_max):
            out.ranges[i] = float('nan')
            continue
        if math.isfinite(distance):
            x, y = 0.32 + distance * math.cos(angle), distance * math.sin(angle)
            if distance < max(msg.range_min, 0.15) or (-0.09 <= x <= 0.43 and abs(y) <= 0.155):
                out.ranges[i] = float('nan')
    # LaserScan stamp denotes its FIRST ray, which changes after cropping.
    ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
    ns += round(first * msg.time_increment * 1e9)
    out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1000000000)
    return out


class FrontScan(Node):
    def __init__(self):
        super().__init__('front_scan')
        self.half_angle = self.declare_parameter('half_angle_deg', 100.0).value
        self.sensor_yaw = self.declare_parameter('sensor_yaw', 0.0).value
        self.pub = self.create_publisher(LaserScan, '/scan_front', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        try:
            self.pub.publish(crop_scan(msg, self.half_angle, self.sensor_yaw))
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = FrontScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
