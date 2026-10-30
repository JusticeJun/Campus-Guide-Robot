"""Small, bounded, passive command/feedback trace; no images or actuator output."""

import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time

from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, RCOut, State
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import Bool, String


class BoundedTrace:
    """Four files of at most 5 MiB each, shared across navigation sessions."""

    def __init__(self, directory, max_bytes=5 * 1024 * 1024, backups=3):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.lock = (directory / '.writer.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise RuntimeError('Another drive trace writer is already running')
        self.handler = RotatingFileHandler(
            directory / 'trace.jsonl', maxBytes=max_bytes, backupCount=backups, encoding='utf-8'
        )
        self.handler.setFormatter(logging.Formatter('%(message)s'))

    def write(self, value):
        # ros messages may contain NaN/Inf; preserve them as JSON strings.
        def clean(item):
            if isinstance(item, float) and not (-float('inf') < item < float('inf')):
                return str(item)
            if isinstance(item, dict):
                return {k: clean(v) for k, v in item.items()}
            if isinstance(item, (tuple, list)):
                return [clean(v) for v in item]
            return item

        line = json.dumps(clean(value), separators=(',', ':'), allow_nan=False)
        if len(line.encode('utf-8')) + 1 >= self.handler.maxBytes:
            raise ValueError('Trace record exceeds file limit')
        # Use the handler's rotation, but propagate IO errors to the node.
        record = logging.LogRecord('drive_trace', logging.INFO, '', 0, line, (), None)
        if self.handler.shouldRollover(record):
            self.handler.doRollover()
        self.handler.stream.write(line + '\n')
        self.handler.flush()

    def close(self):
        self.handler.close()
        self.lock.close()


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def motion(twist):
    return {'vx': twist.linear.x, 'vy': twist.linear.y, 'wz': twist.angular.z}


class DriveTrace(Node):
    def __init__(self):
        super().__init__('drive_trace')
        directory = self.declare_parameter('directory', '').value
        if not directory:
            raise ValueError('drive trace directory is required')
        self.session = self.declare_parameter('session', '').value
        self.trace = BoundedTrace(directory)
        self.samples = {}
        self.failed = False
        self.subscriptions_kept = []
        for topic in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_guarded'):
            self.subscribe(Twist, topic, lambda m: motion(m))
        self.subscribe(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            lambda m: {
                'stamp_ns': stamp_ns(m),
                'vx': m.velocity.x,
                'vy': m.velocity.y,
                'wz': m.yaw_rate,
                'frame': m.coordinate_frame,
                'mask': m.type_mask,
            },
        )
        for topic in ('/odom', '/mavros/local_position/odom'):
            self.subscribe(
                Odometry,
                topic,
                lambda m: {
                    'stamp_ns': stamp_ns(m),
                    **motion(m.twist.twist),
                    'position_variance': [m.pose.covariance[i] for i in (0, 7)],
                    'yaw_variance': m.pose.covariance[35],
                },
            )
        for topic in (
            '/mavros/local_position/velocity_local',
            '/mavros/local_position/velocity_body',
        ):
            self.subscribe(
                TwistStamped, topic, lambda m: {'stamp_ns': stamp_ns(m), **motion(m.twist)}
            )
        self.subscribe(
            OdomInfo,
            '/rtabmap/odom_info_lite',
            lambda m: {
                'stamp_ns': stamp_ns(m),
                'lost': m.lost,
                'inliers': m.inliers,
                'matches': m.matches,
                'time_estimation': m.time_estimation,
            },
        )
        self.subscribe(
            RCOut,
            '/mavros/rc/out',
            lambda m: {'stamp_ns': stamp_ns(m), 'channels': list(m.channels)[:18]},
        )
        self.subscribe(
            State,
            '/mavros/state',
            lambda m: {
                'stamp_ns': stamp_ns(m),
                'connected': m.connected,
                'armed': m.armed,
                'mode': m.mode,
            },
        )
        self.subscribe(
            EstimatorStatus,
            '/mavros/estimator_status',
            lambda m: {
                name: getattr(m, name)
                for name in m.get_fields_and_field_types()
                if name != 'header'
            },
        )
        self.subscribe(Bool, '/navigation/ready', lambda m: {'ready': m.data})
        self.subscribe(
            Bool,
            '/navigation/stop_latched',
            lambda m: {'latched': m.data},
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.subscribe(
            String,
            '/navigation/status',
            lambda m: {'status': m.data[:512]},
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.trace.write(
            {
                'event': 'start',
                'session': self.session,
                'time_ns': time.time_ns(),
                'topics': list(self.samples),
                'note': '10Hz latest samples; rx_age_s is monotonic receipt age; '
                'null means not received; Pixhawk velocity is an estimate',
            }
        )
        self.create_timer(0.1, self.record)
        self.get_logger().info(f'Drive trace: {directory}/trace.jsonl (20 MiB maximum)')

    def subscribe(self, msg_type, topic, extract, qos=qos_profile_sensor_data):
        self.samples[topic] = None

        def receive(msg):
            self.samples[topic] = (time.monotonic(), extract(msg))

        self.subscriptions_kept.append(self.create_subscription(msg_type, topic, receive, qos))

    def record(self):
        if self.failed:
            return
        now = time.monotonic()
        data = {
            topic: None if sample is None else {'rx_age_s': round(now - sample[0], 4), **sample[1]}
            for topic, sample in self.samples.items()
        }
        try:
            self.trace.write(
                {
                    'session': self.session,
                    'time_ns': time.time_ns(),
                    'ros_time_ns': self.get_clock().now().nanoseconds,
                    'samples': data,
                }
            )
        except (OSError, ValueError) as error:
            self.failed = True
            self.get_logger().error(f'Drive trace stopped writing: {error}')

    def destroy_node(self):
        self.trace.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveTrace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
