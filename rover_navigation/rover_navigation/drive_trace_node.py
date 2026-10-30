"""Small, bounded, passive command/feedback trace; no images or actuator output."""

import fcntl
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, RCOut, State
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


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


def resource_sample(proc=Path('/proc')):
    """Small Linux counters, no process scans, images, or control decisions."""
    values = {}
    wanted = {'MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback'}
    for line in (proc / 'meminfo').read_text().splitlines():
        key, rest = line.split(':', 1)
        if key in wanted:
            values[key + '_kib'] = int(rest.split()[0])
    values['load_1m'] = float((proc / 'loadavg').read_text().split()[0])
    for kind in ('cpu', 'memory', 'io'):
        filename = proc / 'pressure' / kind
        if filename.exists():
            for line in filename.read_text().splitlines():
                fields = line.split()
                avg10 = dict(field.split('=') for field in fields[1:]).get('avg10')
                if avg10 is not None:
                    values[f'{kind}_{fields[0]}_avg10_pct'] = float(avg10)
    return values


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
        for topic in ('/odom', '/vo/odom', '/lio/odom', '/mavros/local_position/odom'):
            self.subscribe(
                Odometry,
                topic,
                lambda m: {
                    'stamp_ns': stamp_ns(m),
                    **motion(m.twist.twist),
                    'xyz': [
                        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
                    'quaternion': [
                        getattr(m.pose.pose.orientation, k) for k in ('x', 'y', 'z', 'w')],
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
                # Scalar counters from the existing lite message distinguish
                # moving F2M/bundle-adjustment load from transport delay.
                # Never subscribe to the full feature/point-cloud payload.
                'features': m.features,
                'local_map_size': m.local_map_size,
                'local_key_frames': m.local_key_frames,
                'local_bundle_time': m.local_bundle_time,
                'local_bundle_constraints': m.local_bundle_constraints,
                'key_frame_added': m.key_frame_added,
                'interval': m.interval,
            },
        )
        self.subscribe(
            Info,
            '/rtabmap/info',
            lambda m: {
                'stamp_ns': stamp_ns(m),
                'loop_id': m.loop_closure_id,
                'proximity_id': m.proximity_detection_id,
                'stats': {k: v for k, v in zip(m.stats_keys, m.stats_values) if k in {
                    'Memory/Working_memory_size/', 'Memory/Signatures_retrieved/',
                    'Memory/Signatures_removed/', 'Memory/Local_graph_size/',
                    'Memory/RAM_usage/MB', 'Memory/RAM_estimated/MB',
                    'Timing/Total/ms', 'Timing/Memory_update/ms',
                    'Timing/Reactivation/ms', 'Timing/Map_optimization/ms',
                    'TimingMem/Joining_dictionary_update/ms', 'TimingMem/Add_new_words/ms',
                    'Keypoint/Dictionary_size/words', 'Loop/Visual_inliers/',
                    'Loop/Id/', 'Loop/RejectedHypothesis/',
                    'Loop/Highest_hypothesis_id/', 'Loop/Highest_hypothesis_value/',
                    'Loop/Accepted_hypothesis_id/', 'Loop/Visual_matches/',
                    'Loop/Visual_inliers_ratio/', 'Loop/Optimization_max_error_ratio/',
                    'Loop/Distance_since_last_loc/m',
                    'Proximity/Space_visual_paths_checked/',
                    'Proximity/Space_scan_paths_checked/',
                    'Proximity/Space_detections_added_visually/',
                    'Proximity/Space_detections_added_icp_multi/',
                    'Proximity/Space_last_detection_id/',
                }},
            },
        )
        self.subscribe(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose',
            lambda m: {
                'stamp_ns': stamp_ns(m), 'frame': m.header.frame_id,
                'xyz': [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
                'quaternion': [getattr(m.pose.pose.orientation, k) for k in ('x', 'y', 'z', 'w')],
            },
        )
        # Only retain map->odom: distinguish actual correction changes from
        # ordinary odometry motion or an initial-pose guess. No TF publication.
        self.samples['map_to_odom'] = None
        self.subscriptions_kept.append(self.create_subscription(
            TFMessage, '/tf', self.map_transform, qos_profile_sensor_data))
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
        self.subscribe(Bool, '/external_navigation/ready', lambda m: {'ready': m.data})
        self.subscribe(String, '/external_navigation/status', lambda m: {'status': m.data[:1024]})
        self.subscribe(String, '/localization/fusion_status', lambda m: {'status': m.data[:1024]})
        self.subscribe(OdomInfo, '/localization/odom_info',
                       lambda m: {'stamp_ns': stamp_ns(m), 'lost': m.lost})
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
        self.path_record_at = {}
        for topic in ('/plan', '/received_global_plan'):
            self.subscriptions_kept.append(self.create_subscription(
                NavPath, topic, lambda m, t=topic: self.record_path(t, m),
                qos_profile_sensor_data))
        self.create_timer(0.1, self.record)
        self.create_timer(5.0, self.record_resources)
        self.get_logger().info(f'Drive trace: {directory}/trace.jsonl (20 MiB maximum)')

    def subscribe(self, msg_type, topic, extract, qos=qos_profile_sensor_data):
        self.samples[topic] = None

        def receive(msg):
            self.samples[topic] = (time.monotonic(), extract(msg))

        self.subscriptions_kept.append(self.create_subscription(msg_type, topic, receive, qos))

    def map_transform(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                t = transform.transform
                self.samples['map_to_odom'] = (time.monotonic(), {
                    'stamp_ns': stamp_ns(transform),
                    'xyz': [t.translation.x, t.translation.y, t.translation.z],
                    'quaternion': [getattr(t.rotation, k) for k in ('x', 'y', 'z', 'w')],
                })

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

    def record_path(self, topic, msg):
        now = time.monotonic()
        if self.failed or now - self.path_record_at.get(topic, -1e9) < 1.0:
            return
        self.path_record_at[topic] = now
        stride = max(1, (len(msg.poses) + 79) // 80)
        points = list(msg.poses[::stride])
        if msg.poses and (not points or points[-1] != msg.poses[-1]):
            points.append(msg.poses[-1])
        try:
            self.trace.write({'event': 'path_sample', 'session': self.session,
                              'time_ns': time.time_ns(), 'topic': topic,
                              'frame': msg.header.frame_id, 'stamp_ns': stamp_ns(msg),
                              'original_count': len(msg.poses),
                              'poses': [[p.pose.position.x, p.pose.position.y,
                                         p.pose.orientation.z, p.pose.orientation.w]
                                        for p in points]})
        except (OSError, ValueError):
            pass  # Diagnostic failure must never affect vehicle control.

    def record_resources(self):
        if self.failed:
            return
        try:
            self.trace.write({'event': 'resources', 'session': self.session,
                              'time_ns': time.time_ns(), **resource_sample()})
        except (OSError, ValueError):
            # Diagnostics must never pause/cancel navigation or change FCU mode.
            pass

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
