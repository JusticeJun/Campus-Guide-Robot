"""Opt-in RTAB ExternalNav bridge; preview never reaches the flight controller."""

import fcntl
import json
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
from mavros.mavlink import convert_to_bytes
from mavros_msgs.msg import EstimatorStatus, Mavlink, ParamEvent, State
from mavros_msgs.srv import MessageInterval, ParamPull
from nav_msgs.msg import Odometry
import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as dialect
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rover_navigation.external_navigation import (
    FCU_PROFILE,
    StreamHealth,
    convert_odometry,
    mavlink_packet,
    ros_mavlink,
    stamp_seconds,
)


class ExternalNavigation(Node):
    """Publish measured odometry and a separately checked FCU readiness heartbeat."""

    def __init__(self):
        super().__init__('external_navigation')
        defaults = {
            'transmit': False,
            'mounting_reviewed': False,
            'base_to_fcu_m': [0.25, 0.0, 0.07],
            'mavlink_sink': '/uas1/mavlink_sink',
            'mavlink_source': '/uas1/mavlink_source',
            'minimum_ready_rate_hz': 4.0,
            'max_sample_age_s': 0.6,
            'max_odometry_gap_s': 0.6,
            'reset_file': str(Path.home() / 'ros2_ws/.cache/external_navigation/reset'),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.transmit = self.get_parameter('transmit').value
        self.offset = self.get_parameter('base_to_fcu_m').value
        if self.transmit and not self.get_parameter('mounting_reviewed').value:
            raise ValueError('review FCU mounting before enabling transmission')
        if len(self.offset) != 3 or not np.isfinite(self.offset).all():
            raise ValueError('base_to_fcu_m must contain three finite FLU distances')
        self.lease = None
        self.reset_counter = 0
        if self.transmit:
            # Lifetime lock prevents two bridges; increment durably before any transmission.
            path = Path(self.get_parameter('reset_file').value).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.lease = path.open('a+')
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.lease.seek(0)
            previous = self.lease.read().strip()
            self.reset_counter = (int(previous or '0') + 1) % 256
            self.lease.seek(0)
            self.lease.truncate()
            self.lease.write(str(self.reset_counter))
            self.lease.flush()
            import os

            os.fsync(self.lease.fileno())
        self.codec = dialect.MAVLink(None, srcSystem=1, srcComponent=197)
        self.decoder = dialect.MAVLink(None)
        topic = (
            self.get_parameter('mavlink_sink').value
            if self.transmit
            else '/external_navigation/preview'
        )
        self.output = self.create_publisher(Mavlink, topic, qos_profile_sensor_data)
        self.ready_pub = self.create_publisher(Bool, '/external_navigation/ready', 1)
        self.status_pub = self.create_publisher(String, '/external_navigation/status', 1)
        self.health = StreamHealth(
            max_sample_age_s=self.get_parameter('max_sample_age_s').value,
            max_gap_s=self.get_parameter('max_odometry_gap_s').value,
        )
        self.pending_odom, self.pending_info = {}, {}
        self.state = self.estimator = self.velocity = None
        self.state_at = self.estimator_at = self.velocity_at = -float('inf')
        self.disarmed_seen = self.sent = False
        self.packets_sent = 0
        self.stale_samples_dropped = 0
        self.last_packet_at = None
        self.fault = ''
        self.auto_recoverable = False
        self.coordinate_fault = False
        self.params_ok = False
        self.params_reason = 'waiting for FCU parameter audit'
        self.params_at = -float('inf')
        self.params_future = self.pull_future = None
        self.pulled = False
        self.next_poll = 0.0
        self.request_at = 0.0
        self.feedback_since = None
        self.operational = False
        self.last_status = ''
        self.speed_error = None
        self.params_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.pull_client = self.create_client(ParamPull, '/mavros/param/pull')
        self.interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.intervals = [32, 193]  # LOCAL_POSITION_NED and EKF_STATUS_REPORT.
        self.interval_future = None
        for msg_type, topic, callback in (
            (Odometry, '/odom', self.on_odom),
            (OdomInfo, '/rtabmap/odom_info_lite', self.on_info),
            (State, '/mavros/state', self.on_state),
            (ParamEvent, '/mavros/param/event', self.on_parameter_event),
            (Mavlink, self.get_parameter('mavlink_source').value, self.on_fcu_mavlink),
            (TwistStamped, '/mavros/local_position/velocity_body', self.on_velocity),
        ):
            self.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
        self.create_service(Trigger, '/external_navigation/recover', self.recover)
        self.create_service(Trigger, '/external_navigation/reinitialize', self.reinitialize)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'ExternalNav transmit={self.transmit}; output={self.output.topic_name}'
        )

    def on_fcu_mavlink(self, msg):
        """
        Read ArduRover EKF_STATUS_REPORT, not PX4 ESTIMATOR_STATUS.

        MAVROS sys_status exposes only message 230 on /mavros/estimator_status.
        ArduRover 4.7.1 reports message 193; decode its checked router packet.
        """
        if (msg.sysid, msg.compid, msg.msgid, msg.framing_status) != (
            1,
            1,
            193,
            Mavlink.FRAMING_OK,
        ):
            return
        try:
            report = self.decoder.parse_char(convert_to_bytes(msg))
        except (ValueError, dialect.MAVError):
            return
        if report is None or report.get_type() != 'EKF_STATUS_REPORT':
            return
        flags = report.flags
        self.on_estimator(
            EstimatorStatus(
                attitude_status_flag=bool(flags & 1) and not bool(flags & 1024),
                velocity_horiz_status_flag=bool(flags & 2),
                pos_horiz_rel_status_flag=bool(flags & 8),
                pos_horiz_abs_status_flag=bool(flags & 16),
                const_pos_mode_status_flag=bool(flags & 128),
            )
        )

    def on_state(self, msg):
        """Require a disarmed observation before the first active transmission."""
        self.state, self.state_at = msg, time.monotonic()
        if msg.armed:
            self.auto_recoverable = False
        if msg.connected and not msg.armed:
            self.disarmed_seen = True
        if not msg.connected:
            self.params_ok = self.pulled = self.disarmed_seen = False
            for client, field in (
                (self.params_client, 'params_future'),
                (self.pull_client, 'pull_future'),
            ):
                future = getattr(self, field)
                if future is not None:
                    client.remove_pending_request(future)
                    setattr(self, field, None)
            if self.sent:
                self.fail('FCU disconnected; disarm and recover after reconnection')

    def on_parameter_event(self, msg):
        """Invalidate verified configuration on an explicit relevant FCU change."""
        if msg.param_id not in FCU_PROFILE:
            return
        value = msg.value
        actual = (
            value.integer_value
            if value.type == ParameterType.PARAMETER_INTEGER
            else value.double_value if value.type == ParameterType.PARAMETER_DOUBLE else None
        )
        if (
            actual is None
            or not np.isfinite(actual)
            or abs(actual - FCU_PROFILE[msg.param_id]) > 1e-5
        ):
            self.params_ok = False
            self.params_reason = f'FCU parameter changed: {msg.param_id}={actual}'

    def on_estimator(self, msg):
        """Cache actual FCU EKF status, separately from visual odometry health."""
        self.estimator, self.estimator_at = msg, time.monotonic()

    def on_velocity(self, msg):
        """Cache FCU body FLU velocity for a signed comparison with camera motion."""
        self.velocity, self.velocity_at = msg, time.monotonic()

    def fail(self, reason):
        """Stop bad estimates; allow a disarmed startup interruption to recover."""
        if self.sent and ('discontinuity' in reason or 'timestamp reset' in reason):
            self.coordinate_fault = True
        self.health.reject(reason)
        self.feedback_since = None
        self.operational = False
        if self.sent:
            disarmed = bool(
                self.state and self.state.connected and not self.state.armed
                and time.monotonic() - self.state_at < 2.0
            )
            # Never reclassify an armed/stale-state fault as a startup wait.
            if not self.fault:
                self.auto_recoverable = disarmed
            self.auto_recoverable = (
                self.auto_recoverable and disarmed and not self.coordinate_fault
            )
            self.fault = reason
        self.ready_pub.publish(Bool(data=False))

    def reinitialize(self, request, response):
        """Explicit disarmed new-session recovery; never invoked automatically."""
        return self.recover(request, response, reinitialize=True)

    def recover(self, request, response, reinitialize=False):
        """Resume continuous sensor input while disarmed, never an old navigation goal."""
        now = time.monotonic()
        reason = ''
        if not self.fault:
            reason = 'no ExternalNav fault to recover'
        elif self.coordinate_fault and not reinitialize:
            reason = 'coordinate reset detected; verify localization and restart only the bridge'
        elif not self.state or not self.state.connected or now - self.state_at >= 2.0:
            reason = 'need fresh connected FCU state'
        elif self.state.armed:
            reason = 'disarm before recovering ExternalNav input'
        elif not self.params_ok:
            reason = 'wait for a successful FCU parameter audit'
        elif not self.health.continuous(now):
            reason = 'wait for healthy continuous odometry'
        elif (
            np.linalg.norm(self.health.last.velocity[:2]) > 0.05
            or np.linalg.norm(self.health.last.angular) > 0.05
        ):
            reason = 'odometry still reports motion'
        response.success = not reason
        if reason:
            response.message = reason
            return response
        if self.coordinate_fault:
            # Persist a new MAVLink odometry epoch before admitting any packet.
            # TF/EKF/map and odometry coordinates themselves are NOT reset.
            if not self.lease:
                response.success = False
                response.message = 'no exclusive ExternalNav transmission lease'
                return response
            try:
                import os
                counter = (self.reset_counter + 1) % 256
                self.lease.seek(0)
                self.lease.truncate()
                self.lease.write(str(counter))
                self.lease.flush()
                os.fsync(self.lease.fileno())
            except OSError as error:
                response.success = False
                response.message = 'cannot persist ExternalNav epoch: ' + str(error)
                return response
            self.reset_counter = counter
            self.sent = False
            self.last_packet_at = None
            self.coordinate_fault = False
            self.pending_odom.clear()
            self.pending_info.clear()
            self.health.last = None
            self.health.reject('waiting for fresh odometry after operator recovery')
        self.fault = ''
        self.auto_recoverable = False
        self.feedback_since = None
        self.operational = False
        self.ready_pub.publish(Bool(data=False))
        response.message = (
            'ExternalNav input recovered; wait for FCU readiness, then acknowledge '
            'the navigation stop and submit a NEW goal.'
        )
        return response

    def on_odom(self, msg):
        """Pair only measurements with the exact same RTAB info timestamp."""
        self.pending_odom[stamp_seconds(msg.header.stamp)] = msg
        self.process_pair(stamp_seconds(msg.header.stamp))

    def on_info(self, msg):
        """Invalid samples never transmit; brief loss uses the existing gap limit."""
        if msg.lost:
            self.tracking_loss()
        self.pending_info[stamp_seconds(msg.header.stamp)] = msg
        self.process_pair(stamp_seconds(msg.header.stamp))

    def tracking_loss(self):
        # Fusion's OdomInfo.lost also denotes a temporarily expired input, not
        # necessarily a visual tracking failure. Do not turn a single lost
        # notification into a permanent armed-session transport lock. Neither
        # this notification nor a predicted sample refreshes the watchdog.
        if self.sent and not self.fault and self.health.fresh(time.monotonic()):
            return
        self.fail('odometry input unavailable')

    def process_pair(self, stamp):
        """Send each healthy real sample once; never manufacture timer samples."""
        for cache in (self.pending_odom, self.pending_info):
            while len(cache) > 10:
                del cache[next(iter(cache))]
        if stamp not in self.pending_odom or stamp not in self.pending_info:
            return
        odom, info = self.pending_odom.pop(stamp), self.pending_info.pop(stamp)
        now = time.monotonic()
        if info.lost:
            self.tracking_loss()
            return
        try:
            estimate = convert_odometry(odom, self.offset)
            now_ros = self.get_clock().now().nanoseconds / 1e9
            if now_ros - estimate.stamp > self.health.max_sample_age_s:
                # Drop just this late measurement. The independent valid-input
                # watchdog stops driving if no timely replacement arrives.
                self.stale_samples_dropped += 1
                return
            # Once transmitting, a timestamp gap must not be silently restarted.
            if (
                self.sent
                and not self.fault
                and self.health.last
                and stamp - self.health.last.stamp > self.health.max_gap_s
            ):
                raise ValueError('odometry gap after transmission')
            self.health.accept(estimate, now_ros, now)
        except ValueError as error:
            self.fail(str(error))
            return
        if self.transmit:
            # A disarmed transient startup failure must not starve the FCU of
            # subsequent valid measurements while the readiness window refills.
            if not self.transport_ok(now, allow_startup_fault=True):
                return
        elif self.fault:
            return
        packet = ros_mavlink(mavlink_packet(estimate, self.codec, self.reset_counter))
        packet.header.stamp = odom.header.stamp
        self.output.publish(packet)
        self.sent = self.sent or self.transmit
        if self.transmit:
            self.packets_sent += 1
            self.last_packet_at = now

    def drive_input_ready(self, now):
        """Keep the admission policy separate from observation-only FCU feeding."""
        return self.health.ready(now, self.get_parameter('minimum_ready_rate_hz').value)

    def transport_ok(self, now, allow_startup_fault=False):
        """Check startup authorization and FCU configuration without arming it."""
        return bool(
            self.state
            and self.state.connected
            and now - self.state_at < 2.0
            and self.disarmed_seen
            and self.params_ok
            and (
                not self.fault
                or (
                    allow_startup_fault and self.auto_recoverable
                    and not self.state.armed and not self.coordinate_fault
                )
            )
        )

    def feedback_reason(self, now):
        """Require estimator health and fresh velocity; report speed differences only."""
        e, v = self.estimator, self.velocity
        if e is None or now - self.estimator_at > 1.5:
            return 'waiting for fresh FCU estimator status'
        if not (
            e.attitude_status_flag
            and e.velocity_horiz_status_flag
            and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
            and not e.const_pos_mode_status_flag
            and not e.accel_error_status_flag
        ):
            return 'FCU EKF not ready (check origin and ExternalNav fusion)'
        if v is None or now - self.velocity_at > 1.0:
            return 'waiting for FCU body velocity'
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(v.header.stamp)
        if v.header.frame_id != 'base_link' or not -0.05 <= age <= 1.0:
            return 'invalid FCU velocity frame or timestamp'
        vector = np.array([v.twist.linear.x, -v.twist.linear.y, -v.twist.linear.z])
        if not np.isfinite(vector).all():
            return 'non-finite FCU velocity'
        # Delay and covariance differ across estimators. Report this comparison,
        # but do not stop a healthy EKF on one arbitrary speed-error threshold.
        self.speed_error = float(np.linalg.norm(vector[:2] - self.health.last.velocity[:2]))
        return ''

    def audit_parameters(self, now):
        """Refresh MAVROS once, then read its FCU parameter cache without writes."""
        if not self.state or not self.state.connected:
            return
        if self.pull_future is not None:
            if self.pull_future.done():
                try:
                    self.pulled = bool(self.pull_future.result().success)
                except Exception:
                    self.pulled = False
                self.pull_future = None
            elif now - self.request_at > 20.0:
                self.pull_client.remove_pending_request(self.pull_future)
                self.pull_future = None
                self.next_poll = now + 5.0
            return
        if self.params_future is not None:
            if self.params_future.done():
                try:
                    values = self.params_future.result().values
                    mismatches = []
                    for (name, expected), value in zip(FCU_PROFILE.items(), values):
                        actual = (
                            value.integer_value
                            if value.type == ParameterType.PARAMETER_INTEGER
                            else (
                                value.double_value
                                if value.type == ParameterType.PARAMETER_DOUBLE
                                else None
                            )
                        )
                        if (
                            actual is None
                            or not np.isfinite(actual)
                            or abs(actual - expected) > 1e-5
                        ):
                            mismatches.append(f'{name}={actual} (need {expected})')
                    self.params_ok = len(values) == len(FCU_PROFILE) and not mismatches
                    self.params_reason = '; '.join(mismatches) or 'parameter audit incomplete'
                    self.params_at = now
                except Exception as error:
                    self.params_reason = 'parameter audit retry: ' + str(error)
                self.params_future = None
                self.next_poll = now + 5.0
            elif now - self.request_at > 3.0:
                self.params_client.remove_pending_request(self.params_future)
                self.params_future = None
                self.params_reason = 'parameter audit timed out; retrying'
                self.next_poll = now + 5.0
            return
        if now < self.next_poll:
            return
        self.request_at = now
        if not self.pulled and self.pull_client.service_is_ready():
            self.pull_future = self.pull_client.call_async(ParamPull.Request(force_pull=True))
        elif self.pulled and self.params_client.service_is_ready():
            self.params_future = self.params_client.call_async(
                GetParameters.Request(names=list(FCU_PROFILE))
            )

    def tick(self):
        """Publish fail-closed readiness, including an independent bridge heartbeat."""
        now = time.monotonic()
        self.audit_parameters(now)
        if self.fault and self.auto_recoverable:
            # Same freshness, stationary and continuous-coordinate checks as the
            # explicit recovery service. No goal, arm or mode command is issued.
            self.recover(None, Trigger.Response())
        if self.transmit and self.state and self.state.connected:
            if self.interval_future is not None:
                if self.interval_future.done():
                    self.interval_future = None
            elif self.intervals and self.interval_client.service_is_ready():
                self.interval_future = self.interval_client.call_async(
                    MessageInterval.Request(message_id=self.intervals.pop(0), message_rate=10.0)
                )
        if (
            self.sent
            and not self.fault
            and (not self.health.fresh(now) or not self.transport_ok(now))
        ):
            self.fail('active ExternalNav input/FCU/configuration became unhealthy')
        fcu_reason = self.feedback_reason(now) if self.health.last else 'waiting for odometry'
        # The rate/settling window admits startup, not every instant of an
        # established session. Keep fresh measured feedback flowing through
        # normal arrival jitter; real input/FCU failures still revoke readiness.
        continuing_healthy = bool(
            self.transmit and self.health.fresh(now)
            and self.transport_ok(now) and not fcu_reason
        )
        if not continuing_healthy:
            self.operational = False
        reason = self.fault
        if not reason and not self.operational and not self.drive_input_ready(now):
            minimum = self.get_parameter('minimum_ready_rate_hz').value
            reason = self.health.error or f'need >={minimum:g} Hz fresh odometry for 2 seconds'
        if not reason and not self.transmit:
            reason = 'preview only: no estimates sent to FCU'
        if not reason and not self.transport_ok(now):
            reason = (
                self.params_reason if not self.params_ok else 'need connected disarmed startup'
            )
        if not reason:
            reason = fcu_reason
        if reason:
            self.feedback_since = None
        elif self.feedback_since is None:
            self.feedback_since = now
        ready = not reason and (self.operational or now - self.feedback_since >= 2.0)
        self.operational = bool(ready)
        self.ready_pub.publish(Bool(data=ready))
        status = json.dumps(
            {
                'transmit': self.transmit,
                'input_ready': self.drive_input_ready(now),
                'continuing_healthy': continuing_healthy,
                'input_hz': self.health.rate(),
                'fcu_ready': not fcu_reason,
                'fcu_reason': fcu_reason,
                'packets_sent': self.packets_sent,
                'stale_samples_dropped': self.stale_samples_dropped,
                'last_packet_age_s': (
                    now - self.last_packet_at if self.last_packet_at is not None else None
                ),
                'ready': ready,
                'fault_latched': bool(self.fault),
                'coordinate_fault': self.coordinate_fault,
                'automatic_recovery_pending': bool(self.fault and self.auto_recoverable),
                'reason': reason or ('ready' if ready else 'settling'),
                'reset_counter': self.reset_counter,
                'horizontal_speed_error_m_s': self.speed_error,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(String(data=status))
        status_key = (ready, reason, bool(self.fault))
        if status_key != self.last_status:
            self.get_logger().info(status)
            self.last_status = status_key

    def destroy_node(self):
        """Withdraw readiness; firmware must also retain its own EKF failsafe."""
        self.ready_pub.publish(Bool(data=False))
        if self.lease:
            self.lease.close()
        return super().destroy_node()


def main(args=None):
    """Run an observation-only preview unless transmit is explicitly enabled."""
    rclpy.init(args=args)
    node = ExternalNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
