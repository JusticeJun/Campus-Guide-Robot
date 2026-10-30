"""Measurement validity for local EKF fusion; predictions are not observations."""

import math


def lidar_velocity_valid(twist, visual_twist=None):
    """
    Reject ICP motion inconsistent with a rear-axle Ackermann base frame.

    These are measurement checks, not command limits. A missing camera still
    permits forward/reverse lidar tracking; gyro alone never sustains tracking.
    """
    if abs(twist.linear.y) > 0.20:
        return False
    if visual_twist is not None and math.hypot(
            twist.linear.x-visual_twist.linear.x,
            twist.linear.y-visual_twist.linear.y) > 0.50:
        return False
    return True


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def valid_motion(msg, now, max_age=1.5):
    if not - 0.1 <= now - seconds(msg.header.stamp) <= max_age:
        return False
    if msg.child_frame_id != 'base_link':
        return False
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        return False
    if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.01:
        return False
    twist = msg.twist.twist
    values = (twist.linear.x, twist.linear.y, twist.angular.z)
    if not all(math.isfinite(v) for v in values):
        return False
    # Gross invalid estimates, not a command-speed limiter.
    if math.hypot(*values[:2]) > 3.0 or abs(values[2]) > 3.0:
        return False
    for matrix in (msg.pose.covariance, msg.twist.covariance):
        if not all(math.isfinite(v) for v in matrix):
            return False
        if any(matrix[i] < 0 or matrix[i] >= 9999 for i in (0, 7, 35)):
            return False
    return True


class MeasurementHealth:
    """Either independent odometry can sustain fusion, never gyro alone."""

    def __init__(self, timeout=1.5):
        self.timeout = timeout
        self.samples = {}

    def update(self, source, stamp, valid):
        previous = self.samples.get(source)
        if previous and stamp <= previous[0]:
            return False
        self.samples[source] = (stamp, valid)
        return True

    def active(self, now):
        return [name for name, (stamp, valid) in self.samples.items()
                if valid and - 0.1 <= now - stamp <= self.timeout]
