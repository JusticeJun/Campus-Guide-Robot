"""Continuous RTAB odometry to ArduPilot LOCAL_FRD/BODY_FRD estimates."""

from collections import deque
from dataclasses import dataclass
import math
import struct

import numpy as np


# No actuator tuning or arming-check bypass belongs in this profile.
FCU_PROFILE = {
    'AHRS_EKF_TYPE': 3,
    'EK3_ENABLE': 1,
    'EK3_SRC_OPTIONS': 0,
    'FS_EKF_ACTION': 1,
    'FS_EKF_THRESH': 0.8,
    'EK3_SRC1_POSXY': 6,
    'EK3_SRC1_VELXY': 6,
    'EK3_SRC1_POSZ': 1,
    'EK3_SRC1_VELZ': 0,
    'EK3_SRC1_YAW': 6,
    'EK3_SRC2_POSXY': 0,
    'EK3_SRC2_VELXY': 0,
    'EK3_SRC2_YAW': 0,
    'EK3_SRC3_POSXY': 0,
    'EK3_SRC3_VELXY': 0,
    'EK3_SRC3_YAW': 0,
    'GPS1_TYPE': 0,
    'GPS2_TYPE': 0,
    'COMPASS_USE': 0,
    'COMPASS_USE2': 0,
    'COMPASS_USE3': 0,
    'VISO_TYPE': 3,
    'VISO_QUAL_MIN': 1,
    'VISO_SCALE': 1,
    'VISO_ORIENT': 0,
    # Bridge reports the FCU reference point, not the camera optical center.
    'VISO_POS_X': 0,
    'VISO_POS_Y': 0,
    'VISO_POS_Z': 0,
}


def stamp_seconds(stamp):
    """Read a ROS timestamp without using receipt time as measurement time."""
    return stamp.sec + stamp.nanosec * 1e-9


def skew(vector):
    """Return the cross-product matrix."""
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation(q):
    """Return body-to-world rotation for a normalized ROS xyzw quaternion."""
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or abs(float(q @ q) - 1.0) > 0.01:
        raise ValueError('invalid quaternion')
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def covariance(values):
    """Reject unknown/invalid covariance and retain cross correlations."""
    cov = np.asarray(values, dtype=float).reshape(6, 6)
    if (
        not np.isfinite(cov).all()
        or not np.allclose(cov, cov.T, atol=1e-7)
        or np.linalg.eigvalsh(cov).min() < -1e-7
        or np.diag(cov).max() > 1.0
    ):
        raise ValueError('invalid or excessive covariance')
    # RTAB registration covariance is not calibrated global uncertainty.
    # Never advertise exact measurements, including planar zero variances.
    return cov + np.diag([0.01] * 3 + [0.0025] * 3)


@dataclass
class Estimate:
    """One validated estimate at the FCU point in MAVLink FRD frames."""

    stamp: float
    position: np.ndarray
    quaternion: list
    velocity: np.ndarray
    angular: np.ndarray
    pose_covariance: list
    velocity_covariance: list


def convert_odometry(msg, offset):
    """Move base_link to FCU center, then convert both FLU axes to FRD."""
    if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
        raise ValueError('expected odom -> base_link; map corrections are forbidden')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    pos = np.array([p.x, p.y, p.z])
    vel = np.array([v.x, v.y, v.z])
    omega = np.array([w.x, w.y, w.z])
    lever = np.asarray(offset, dtype=float)
    if lever.shape != (3,) or not np.isfinite(np.r_[pos, vel, omega, lever]).all():
        raise ValueError('non-finite odometry or mounting offset')
    if np.linalg.norm(vel) > 1.5 or np.linalg.norm(omega) > 2.0:
        raise ValueError('odometry exceeds low-speed rover envelope')
    quat = np.array([q.x, q.y, q.z, q.w])
    rot = rotation(quat)
    quat /= np.linalg.norm(quat)
    d = np.diag([1.0, -1.0, -1.0])
    axes = np.diag([1.0, -1.0, -1.0, 1.0, -1.0, -1.0])
    pose_j, twist_j = np.eye(6), np.eye(6)
    pose_j[:3, 3:] = -skew(rot @ lever)
    twist_j[:3, 3:] = -skew(lever)
    pcov = axes @ pose_j @ covariance(msg.pose.covariance) @ pose_j.T @ axes
    vcov = axes @ twist_j @ covariance(msg.twist.covariance) @ twist_j.T @ axes
    indices = np.triu_indices(6)
    return Estimate(
        stamp_seconds(msg.header.stamp),
        d @ (pos + rot @ lever),
        [quat[3], quat[0], -quat[1], -quat[2]],
        d @ (vel + np.cross(omega, lever)),
        d @ omega,
        pcov[indices].tolist(),
        vcov[indices].tolist(),
    )


class StreamHealth:
    """Require unique fresh samples; detect gaps and coordinate resets."""

    def __init__(self, max_sample_age_s=0.6, max_gap_s=0.6):
        if not 0 < max_sample_age_s <= 1.0 or not 0 < max_gap_s <= 2.0:
            raise ValueError('odometry age must be within (0, 1] and gap within (0, 2] seconds')
        self.max_sample_age_s = max_sample_age_s
        self.max_gap_s = max_gap_s
        self.samples = deque(maxlen=120)
        self.continuous_since = None
        self.last = None
        self.error = 'waiting for synchronized odometry'

    def reject(self, reason):
        """Invalidate readiness without keeping a previous good window."""
        self.samples.clear()
        self.continuous_since = None
        self.error = reason

    def accept(self, estimate, now_ros, now_mono):
        """Validate timestamps and physical continuity before admitting a sample."""
        if not -0.05 <= now_ros - estimate.stamp <= self.max_sample_age_s:
            raise ValueError('stale or future odometry')
        if self.last is not None:
            dt = estimate.stamp - self.last.stamp
            if dt <= 0:
                raise ValueError('odometry timestamp reset or reordered sample')
            if dt > self.max_gap_s:
                self.reject('odometry gap')
            if np.linalg.norm(estimate.position - self.last.position) > 0.15 + dt:
                raise ValueError('odometry position discontinuity')
            dot = abs(np.dot(estimate.quaternion, self.last.quaternion))
            angle = 2 * math.acos(min(1.0, dot))
            if angle > 0.15 + 1.5 * dt:
                raise ValueError('odometry attitude discontinuity')
        if self.samples and now_mono - self.samples[-1] > self.max_gap_s:
            self.reject('odometry reception gap')
        if self.continuous_since is None:
            self.continuous_since = now_mono
        self.samples.append(now_mono)
        while len(self.samples) > 1 and now_mono - self.samples[0] > 2.5:
            self.samples.popleft()
        self.last = estimate
        self.error = ''

    def fresh(self, now):
        """Check whether a validated sample can be sent before the startup window fills."""
        return bool(
            not self.error and self.samples
            and now - self.samples[-1] <= self.max_gap_s
        )

    def rate(self):
        """Observed arrival rate, not a manufactured publication frequency."""
        if len(self.samples) < 2:
            return 0.0
        span = self.samples[-1] - self.samples[0]
        return (len(self.samples) - 1) / span if span > 0 else 0.0

    def continuous(self, now):
        """Two seconds of valid data without a timestamp/reception gap."""
        return bool(
            self.fresh(now) and len(self.samples) >= 3
            and self.continuous_since is not None
            and self.samples[-1] - self.continuous_since >= 2.0
        )

    def ready(self, now, minimum_hz=4.0):
        """Drive admission is distinct from sending a valid measurement."""
        if not np.isfinite(minimum_hz) or not 2.0 <= minimum_hz <= 30.0:
            return False
        return self.continuous(now) and self.rate() >= minimum_hz


def mavlink_packet(estimate, codec, reset_counter):
    """Encode ODOMETRY with generated MAVLink v2 dialect, including CRC."""
    from pymavlink.dialects.v20 import ardupilotmega as dialect

    message = dialect.MAVLink_odometry_message(
        int(round(estimate.stamp * 1e6)),
        dialect.MAV_FRAME_LOCAL_FRD,
        dialect.MAV_FRAME_BODY_FRD,
        *estimate.position,
        estimate.quaternion,
        *estimate.velocity,
        *estimate.angular,
        estimate.pose_covariance,
        estimate.velocity_covariance,
        reset_counter,
        dialect.MAV_ESTIMATOR_TYPE_VISION,
        1,
    )
    packet = bytes(message.pack(codec))
    codec.seq = (codec.seq + 1) % 256
    return packet


def ros_mavlink(packet):
    """Wrap an already encoded MAVLink2 packet for the MAVROS router sink."""
    from mavros_msgs.msg import Mavlink

    size = packet[1]
    payload = packet[10:10 + size]
    padded = payload + b'\0' * ((-size) % 8)
    return Mavlink(
        framing_status=Mavlink.FRAMING_OK,
        magic=packet[0],
        len=size,
        incompat_flags=packet[2],
        compat_flags=packet[3],
        seq=packet[4],
        sysid=packet[5],
        compid=packet[6],
        msgid=int.from_bytes(packet[7:10], 'little'),
        checksum=int.from_bytes(packet[10 + size:12 + size], 'little'),
        payload64=list(struct.unpack('<' + 'Q' * (len(padded) // 8), padded)),
    )
