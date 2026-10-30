"""Headless readiness, stationary localization recovery and hard safety stops."""

import json
import math
from pathlib import Path as FilePath

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage
import yaml

from rover_navigation.path_validation import path_is_clear, shared_footprint


def valid_estimate(msg, frame, position_variance, yaw_variance):
    pose = msg.pose.pose
    values = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        *msg.pose.covariance,
    ]
    norm = sum(x * x for x in values[3:7])
    return (
        msg.header.frame_id == frame
        and all(math.isfinite(x) for x in values)
        and abs(norm - 1.0) < 0.01
        and all(0 < msg.pose.covariance[i] <= position_variance for i in (0, 7))
        and 0 < msg.pose.covariance[35] <= yaw_variance
    )


def valid_transform(msg):
    translation, rotation = msg.transform.translation, msg.transform.rotation
    values = [
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    ]
    return (
        all(math.isfinite(x) for x in values) and abs(sum(x * x for x in values[3:]) - 1.0) < 0.01
    )


def valid_native_plan(msg):
    """Nav2 checks collision/footprint; retain frame and numeric validity here."""
    if msg.header.frame_id != 'map' or not msg.poses:
        return False
    for item in msg.poses:
        p, q = item.pose.position, item.pose.orientation
        if (
            item.header.frame_id not in ('', 'map')
            or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) >= 0.01
        ):
            return False
    return True


class LocalizationGuard(Node):
    """Guard output upstream of the unchanged Pixhawk command watchdog."""

    def __init__(self):
        super().__init__('localization_guard')
        defaults = FilePath(get_package_share_directory('rover_navigation')) / 'config/nav2_rtab.yaml'
        footprint, padding = shared_footprint(yaml.safe_load(defaults.read_text()))
        for key, value in {
            # Deprecated compatibility parameter; RTAB events have no age deadline.
            'localization_timeout_s': 2.0,
            'odometry_timeout_s': 1.0,
            'maximum_position_variance_m2': 0.25,
            'maximum_yaw_variance_rad2': 0.12,
            'stable_duration_s': 3.0,
            'path_timeout_s': 4.0,
            'footprint_padding': float(padding),
            'footprint': json.dumps(footprint),
            'goal_source': 'route',
            'occupancy_topic': '/navigation/route_map',
            'coarse_hint_verified_ready_timeout_s': 2.5,
            'coarse_hint_runtime_odom_tf_timeout_s': 2.0,
        }.items():
            self.declare_parameter(key, value)
        self.coarse_hint_gate_enabled = self.declare_parameter(
            'enable_coarse_hint_handoff', False, ParameterDescriptor(read_only=True)
        ).value
        self.native_localization_only = self.declare_parameter(
            'native_localization_only', False, ParameterDescriptor(read_only=True)
        ).value
        self.native_localization_initialized = False
        self.strict_tracking_safety = self.declare_parameter(
            'strict_tracking_safety', False, ParameterDescriptor(read_only=True)
        ).value
        self.tracking_max_age_s = self.declare_parameter(
            'tracking_max_age_s', 2.5, ParameterDescriptor(read_only=True)
        ).value
        if not math.isfinite(self.tracking_max_age_s) or self.tracking_max_age_s <= 0:
            raise ValueError('tracking_max_age_s must be finite and positive')
        self.odom_info = self.odom_info_time = None
        # Used when replacing only this node after a tracking stop. Preserve the
        # stop until normal health/cancellation/stationary acknowledgement passes.
        self.tracking_stop_latched = self.declare_parameter(
            'start_tracking_stop_latched', False, ParameterDescriptor(read_only=True)
        ).value
        self.goal_transition_pending = False
        for name in (
            'coarse_hint_verified_ready_timeout_s',
            'coarse_hint_runtime_odom_tf_timeout_s',
        ):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        self.coarse_hint_verified_ready = False
        self.coarse_hint_verified_ready_time = None
        self.runtime_safety_reason = self.localization_ready_reason = ''
        self.final_navigation_ready = False
        self.runtime_stable = False
        self.footprint = json.loads(self.get_parameter('footprint').value)
        self.loc = self.odom = self.command = self.command_time = None
        self.odom_time = None
        self.odom_rejection = ''
        self.rtab_info = self.rtab_info_time = None
        self.route_map = self.path_time = self.route_time = None
        self.path_clear = False
        self.ever_ready = False
        self.latched = self.tracking_stop_latched
        self.recovering = False
        self.resume_after = None
        self.recovery_started = None
        self.recovery_cause = ''
        self.latch_reason = (
            'tracking stop preserved across guard restart' if self.tracking_stop_latched else ''
        )
        self.goal_outcome = ''
        self.tracked_goal_id = None
        self.stable_since = None
        self.last_reason = ''
        self.goal_source = self.get_parameter('goal_source').value
        if self.goal_source not in ('route', 'nav2'):
            raise ValueError('goal_source must be route or nav2')
        self.active_goals = set()
        self.goal_started = None
        self.cancel_request = None
        self.tf_parent_conflict = False
        self.invalid_tf = False
        self.invalid_tf_frames = set()
        self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_guarded', 10)
        self.ready_pub = self.create_publisher(Bool, '/navigation/ready', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stop_latched_pub = self.create_publisher(Bool, '/navigation/stop_latched', qos)
        self.create_subscription(TFMessage, '/tf_static', self.tf_callback, qos)
        self.status_pub = self.create_publisher(String, '/navigation/status', qos)
        if self.coarse_hint_gate_enabled:
            self.coarse_hint_ready_sub = self.create_subscription(
                Bool,
                '/localization/coarse_hint/verified_ready',
                self.coarse_hint_ready_callback,
                1,
            )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.localization_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Info, '/rtabmap/info', self.info_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/odom', self.odometry_callback, qos_profile_sensor_data
        )
        if self.strict_tracking_safety:
            # Lite preserves tracking status/timing without transporting the
            # feature arrays and local map used only by visualization tools.
            self.create_subscription(
                OdomInfo,
                '/rtabmap/odom_info_lite',
                self.odom_info_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('occupancy_topic').value, self.map_callback, qos
        )
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        if self.goal_source == 'route':
            self.create_subscription(Bool, '/navigation/route_active', self.route_callback, 1)
        else:
            self.create_subscription(
                GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, qos
            )
            self.cancel_client = self.create_client(
                CancelGoal, '/navigate_to_pose/_action/cancel_goal'
            )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('controller_server', 'planner_server', 'bt_navigator')
        }
        self.states = {}
        self.requests = {}
        self.request_times = {}
        self.create_service(Trigger, '/navigation/stop', self.stop_callback)
        self.create_service(
            Trigger, '/navigation/acknowledge_tracking_stop', self.acknowledge_tracking_stop
        )
        self.create_timer(0.5, self.poll_lifecycle)
        self.create_timer(0.05, self.tick)

    def stop_callback(self, request, response):
        self.latched = True
        self.tracking_stop_latched = False
        self.latch_reason = 'operator stop'
        self.ready_pub.publish(Bool(data=False))
        self.report('stop_latched: operator stop')
        self.publisher.publish(Twist())
        response.success = True
        response.message = 'Stopped; restart navigation to clear latch.'
        return response

    def acknowledge_tracking_stop(self, request, response):
        """Operator acknowledgement never resumes the old goal or command."""
        reason = self.health_reason()
        if not self.tracking_stop_latched:
            reason = 'no tracking stop to acknowledge (other stops require restart)'
        elif self.goal_is_active() or self.goal_transition_pending:
            reason = 'wait for the previous goal to finish cancelling'
        elif getattr(self, 'cancel_request', None) is not None and not self.cancel_request.done():
            reason = 'wait for the pending cancellation response'
        elif not reason and not self.runtime_stable:
            reason = 'wait for stable localization'
        elif not reason:
            twist = self.odom.twist.twist
            if math.hypot(twist.linear.x, twist.linear.y) > 0.05 or abs(twist.angular.z) > 0.05:
                reason = 'odometry still reports motion'
        response.success = not reason
        if reason:
            response.message = reason
            return response
        self.latched = self.tracking_stop_latched = False
        self.latch_reason = ''
        self.command = self.command_time = self.path_time = None
        self.path_clear = False
        self.stable_since = None
        self.resume_after = self.get_clock().now()
        self.publisher.publish(Twist())
        response.message = 'Tracking stop acknowledged; wait for ready, then submit a NEW goal.'
        self.tick()
        return response

    def odom_info_callback(self, msg):
        self.odom_info = msg
        self.odom_info_time = self.get_clock().now()
        self.tick()  # Explicit lost must stop in this callback, without the odom grace period.

    def tracking_health_reason(self):
        if self.odom_rejection:
            return self.odom_rejection
        if self.odom_info is None:
            return 'waiting_odom_info'
        if self.odom_info.lost:
            return 'visual_odometry_lost'
        if not self.recent(self.odom_info_time, self.get_parameter('odometry_timeout_s').value):
            return 'stale_odom_info'
        for msg in (self.odom_info, self.odom):
            if msg is not None and not self.fresh(
                msg.header.stamp, self.tracking_max_age_s, future_tolerance=0.2
            ):
                return 'stale_tracking_stamp'
        return ''

    def tf_callback(self, msg):
        expected = {'odom': 'map', 'base_link': 'odom', 'camera_link': 'base_link'}
        for tf in msg.transforms:
            if tf.child_frame_id in expected and tf.header.frame_id != expected[tf.child_frame_id]:
                self.tf_parent_conflict = True
            if tf.child_frame_id in expected:
                # tf2 may discard malformed updates and retain an older valid TF.
                if valid_transform(tf):
                    self.invalid_tf_frames.discard(tf.child_frame_id)
                else:
                    self.invalid_tf_frames.add(tf.child_frame_id)
                self.invalid_tf = bool(self.invalid_tf_frames)
        if self.invalid_tf or self.tf_parent_conflict:
            self.tick()

    def poll_lifecycle(self):
        if self.goal_source == 'nav2' and self.latched:
            # Retry while latched, including goals submitted after an operator stop.
            if self.cancel_client.service_is_ready() and (
                self.cancel_request is None or self.cancel_request.done()
            ):
                self.cancel_request = self.cancel_client.call_async(CancelGoal.Request())
        for name, client in self.lifecycle_clients.items():
            if name in self.requests:
                if self.recent(self.request_times[name], 1.5):
                    continue
                expired = self.requests.pop(name)
                expired.cancel()
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self.requests[name] = future
            self.request_times[name] = self.get_clock().now()
            future.add_done_callback(lambda result, key=name: self.lifecycle_result(key, result))

    def lifecycle_result(self, name, future):
        if self.requests.get(name) is not future:
            return  # A delayed response must not overwrite a newer request/state.
        self.requests.pop(name, None)
        self.request_times.pop(name, None)
        try:
            self.states[name] = (future.result().current_state.id, self.get_clock().now())
        except Exception:
            self.states.pop(name, None)

    def localization_callback(self, msg):
        # Optional registration evidence, not an initialization prerequisite.
        # Retain invalid estimates too; an explicit invalid event revokes readiness.
        self.loc = msg
        if getattr(self, 'native_localization_only', False):
            # With pub_loc_pose_only_when_localizing=true, this event is RTAB's
            # accepted registration. Do not impose another quality threshold.
            if msg is not None and valid_estimate(msg, 'map', math.inf, math.inf):
                self.native_localization_initialized = True
            return
        if getattr(self, 'coarse_hint_gate_enabled', False):
            return  # Global correctness belongs exclusively to the verifier.
        if msg is not None and not valid_estimate(
            msg,
            'map',
            self.get_parameter('maximum_position_variance_m2').value,
            self.get_parameter('maximum_yaw_variance_rad2').value,
        ):
            self.tick()

    def coarse_hint_ready_callback(self, msg):
        self.coarse_hint_verified_ready = bool(msg.data)
        self.coarse_hint_verified_ready_time = self.get_clock().now()
        self.tick()  # A false verdict revokes output in this callback, not next timer.

    def coarse_hint_localization_reason(self):
        stamp = self.coarse_hint_verified_ready_time
        if stamp is None:
            return 'missing_coarse_hint_verified_ready'
        if not self.recent(
            stamp, self.get_parameter('coarse_hint_verified_ready_timeout_s').value
        ):
            return 'stale_coarse_hint_verified_ready'
        if not self.coarse_hint_verified_ready:
            return 'coarse_hint_verified_ready_false'
        return ''

    def info_callback(self, msg):
        # Auxiliary event data only; Info publication is not a liveness heartbeat.
        self.rtab_info, self.rtab_info_time = msg, self.get_clock().now()
        if getattr(self, 'native_localization_only', False) and (
            msg.loop_closure_id > 0 or msg.proximity_detection_id > 0 or msg.landmark_id != 0
        ):
            self.native_localization_initialized = True

    def odometry_callback(self, msg):
        # Only accepted data refreshes the existing receipt watchdog. An invalid
        # burst is equivalent to a gap in usable estimates, not a fresh estimate.
        self.odom_rejection = self.odometry_reason(msg)
        if not self.odom_rejection:
            self.odom, self.odom_time = msg, self.get_clock().now()
        if self.odom_rejection or self.health_reason():
            self.tick()  # Other health failures still stop without a grace period.

    def odometry_reason(self, msg):
        if msg is None:
            return 'waiting_odom_localization'
        if msg.header.frame_id != 'odom':
            return 'invalid_odom_frame'
        if msg.child_frame_id != 'base_link':
            return 'invalid_odometry_child_frame'
        if not valid_estimate(
            msg,
            'odom',
            self.get_parameter('maximum_position_variance_m2').value,
            self.get_parameter('maximum_yaw_variance_rad2').value,
        ):
            return 'invalid_odom_pose_or_covariance'
        if not all(
            math.isfinite(v)
            for v in (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            )
        ):
            return 'invalid_odometry_twist'
        return ''

    def command_callback(self, msg):
        if self.recovering or self.latched or not self.path_clear:
            return
        self.command, self.command_time = msg, self.get_clock().now()

    def route_callback(self, msg):
        self.route_time = self.get_clock().now() if msg.data else None

    def goal_status_callback(self, msg):
        """Observe the common Nav2 action, independently of which client sent it."""
        executing = {
            bytes(s.goal_info.goal_id.uuid)
            for s in msg.status_list
            if s.status == GoalStatus.STATUS_EXECUTING
        }
        pending = any(
            s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_CANCELING)
            for s in msg.status_list
        )
        self.goal_transition_pending = pending
        # Preemption/cancellation cannot keep an old path or command enabled.
        active = executing if len(executing) == 1 and not pending else set()
        if active:
            self.tracked_goal_id = next(iter(active))
            self.goal_outcome = ''
        terminal = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        for status in msg.status_list:
            if (
                bytes(status.goal_info.goal_id.uuid) == self.tracked_goal_id
                and status.status in terminal
            ):
                self.goal_outcome = terminal[status.status]
                self.tracked_goal_id = None
        if active != self.active_goals:
            self.path_clear = False
            self.path_time = self.command_time = None
            self.goal_started = None
            if active:
                accepted = next(
                    s.goal_info.stamp
                    for s in msg.status_list
                    if bytes(s.goal_info.goal_id.uuid) in active
                )
                stamp = Time.from_msg(accepted)
                self.goal_started = stamp if stamp.nanoseconds > 0 else self.get_clock().now()
            self.publisher.publish(Twist())
        self.active_goals = active

    def map_callback(self, msg):
        self.route_map = msg
        self.path_clear = False
        self.path_time = None

    def path_callback(self, msg):
        if (
            self.recovering
            or self.latched
            or (
                self.resume_after is not None
                and Time.from_msg(msg.header.stamp) <= self.resume_after
            )
        ):
            return
        if self.goal_source == 'nav2' and (
            self.goal_started is None or Time.from_msg(msg.header.stamp) < self.goal_started
        ):
            return
        if self.route_map is not None and Time.from_msg(msg.header.stamp) < Time.from_msg(
            self.route_map.header.stamp
        ):
            return  # Delayed plan for a previous route; never reactivate it.
        if not self.path_clear:
            self.command_time = None  # Require a command received after the new plan.
        if getattr(self, 'native_localization_only', False):
            self.path_clear = valid_native_plan(msg)
        else:
            self.path_clear = (
                msg.header.frame_id == 'map'
                and self.route_map is not None
                and Time.from_msg(msg.header.stamp) >= Time.from_msg(self.route_map.header.stamp)
                and path_is_clear(
                    msg.poses,
                    self.route_map,
                    self.footprint,
                    self.get_parameter('footprint_padding').value,
                )
            )
        self.path_time = Time.from_msg(msg.header.stamp)
        if not self.path_clear and self.goal_is_active():
            self.latched = True
            self.latch_reason = (
                'invalid_nav2_path'
                if getattr(self, 'native_localization_only', False)
                else 'path intersects occupied/unknown/outside route map'
            )
            self.report('stop_latched: ' + self.latch_reason)
            self.ready_pub.publish(Bool(data=False))
            self.publisher.publish(Twist())

    def fresh(self, stamp, timeout, future_tolerance=0.0):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return -future_tolerance <= age <= timeout

    def health_reason(self):
        runtime = self.runtime_health_reason()
        if not getattr(self, 'coarse_hint_gate_enabled', False):
            return runtime
        self.runtime_safety_reason = runtime
        self.localization_ready_reason = self.coarse_hint_localization_reason()
        return runtime or self.localization_ready_reason

    def runtime_health_reason(self):
        gated = getattr(self, 'coarse_hint_gate_enabled', False)
        native = getattr(self, 'native_localization_only', False)
        if getattr(self, 'strict_tracking_safety', False):
            reason = self.tracking_health_reason()
            if reason:
                return reason
        if native and not self.native_localization_initialized:
            return 'waiting_native_localization'
        if gated and self.odom_rejection:
            return self.odom_rejection  # Latest invalid odom cannot authorize movement.
        if self.odom_rejection in ('invalid_odom_frame', 'invalid_odometry_child_frame'):
            return self.odom_rejection  # Do not mask a structural mismatch with cached data.
        for msg, frame in (
            (self.loc, 'map'),
            (self.odom, 'odom'),
        ):
            if (gated or native) and frame == 'map':
                continue  # No second global pose/covariance threshold after verdict.
            if frame == 'map' and msg is None:
                continue  # RTAB's live map->odom TF is checked below on every tick.
            if msg is None:
                return f'waiting_{frame}_localization'
            if msg.header.frame_id != frame:
                return f'invalid_{frame}_frame'
            if not valid_estimate(
                msg,
                frame,
                self.get_parameter('maximum_position_variance_m2').value,
                self.get_parameter('maximum_yaw_variance_rad2').value,
            ):
                return f'invalid_{frame}_pose_or_covariance'
        # Establish and sustain readiness with live TF/odometry and Nav2 state.
        # Neither RTAB event topic must have published since this guard started.
        if not self.recent(self.odom_time, self.get_parameter('odometry_timeout_s').value):
            return 'stale_odom_localization'
        if self.odom.child_frame_id != 'base_link':
            return 'invalid_odometry_child_frame'
        if not all(
            math.isfinite(v)
            for v in (
                self.odom.twist.twist.linear.x,
                self.odom.twist.twist.linear.y,
                self.odom.twist.twist.linear.z,
                self.odom.twist.twist.angular.x,
                self.odom.twist.twist.angular.y,
                self.odom.twist.twist.angular.z,
            )
        ):
            return 'invalid_odometry_twist'
        if self.tf_parent_conflict:
            return 'conflicting_tf_parent'
        if self.invalid_tf:
            if 'camera_link' in self.invalid_tf_frames:
                return 'invalid_tf_base_link_camera_link'
            return 'invalid_tf_transform'
        try:
            # Static extrinsics have no freshness deadline, but must be connected.
            camera_tf = self.tf_buffer.lookup_transform('base_link', 'camera_link', Time())
            if not valid_transform(camera_tf):
                return 'invalid_tf_base_link_camera_link'
            for parent, child in (('map', 'odom'), ('odom', 'base_link')):
                tf = self.tf_buffer.lookup_transform(parent, child, Time())
                if not valid_transform(tf):
                    return f'invalid_tf_{parent}_{child}'
                # RTAB 0.23.7 broadcasts map->odom independently at 20 Hz,
                # stamped now()+0.1 s, even with an unchanged map correction.
                # This watches that broadcaster, NOT visual registration age.
                # RGB-D odom TF instead carries the image acquisition stamp;
                # its producer's liveness is already checked via /odom receipts.
                if parent == 'map' and not self.fresh(tf.header.stamp, 0.5, future_tolerance=0.2):
                    return f'stale_tf_{parent}_{child}'
                if (
                    gated
                    and parent == 'odom'
                    and not self.fresh(
                        tf.header.stamp,
                        self.get_parameter('coarse_hint_runtime_odom_tf_timeout_s').value,
                        future_tolerance=0.2,
                    )
                ):
                    return f'stale_tf_{parent}_{child}'
            # Separate latest-edge lookups can succeed without overlapping
            # histories. Require the composed chain used by Nav2 as well.
            if not valid_transform(self.tf_buffer.lookup_transform('map', 'base_link', Time())):
                return 'invalid_tf_map_base_link'
        except TransformException:
            return 'missing_tf'
        for name in self.lifecycle_clients:
            state, stamp = self.states.get(name, (None, None))
            if stamp is None or not self.recent(stamp, 1.5):
                return f'stale_lifecycle_{name}'
            if state != 3:
                return f'waiting_active_{name}'
        if gated and self.goal_is_active() and self.path_clear:
            # Loss of an established control stream pauses the subtree. Missing
            # post-recovery plan/command is allowed so the BT can generate them.
            if not self.recent(self.path_time, self.get_parameter('path_timeout_s').value):
                return 'stopped_stale_controller_path'
            if self.command is not None and not all(
                math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z)
            ):
                return 'stopped_invalid_controller_command'
            if self.command_time is not None and not self.recent(self.command_time, 0.3):
                return 'stopped_stale_controller_command'
        return ''

    def recent(self, stamp, timeout):
        return (
            stamp is not None
            and 0 <= (self.get_clock().now() - stamp).nanoseconds / 1e9 <= timeout
        )

    def goal_is_active(self):
        if self.goal_source == 'nav2':
            return bool(self.active_goals)
        return self.recent(self.route_time, 0.5)

    def report(self, reason):
        if getattr(self, 'coarse_hint_gate_enabled', False):
            stamp = self.coarse_hint_verified_ready_time
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9 if stamp is not None else None
            localization_reason = self.coarse_hint_localization_reason()
            runtime_reason = self.runtime_safety_reason
            if self.latched:
                runtime_reason = 'stop_latched: ' + self.latch_reason
            status = String(
                data=json.dumps(
                    {
                        'status': reason,
                        'coarse_hint_gate_enabled': True,
                        'coarse_hint_verified_ready': self.coarse_hint_verified_ready,
                        'coarse_hint_verified_ready_age': age,
                        'localization_ready_source': 'coarse_hint_verified_ready',
                        'localization_ready': not localization_reason,
                        'runtime_safety_ready': not runtime_reason and self.runtime_stable,
                        'runtime_safety_reason': runtime_reason
                        or ('' if self.runtime_stable else 'waiting_stable_runtime_safety'),
                        'final_navigation_ready': self.final_navigation_ready and not self.latched,
                        'not_ready_reason': runtime_reason
                        or localization_reason
                        or (
                            '' if self.final_navigation_ready else 'waiting_stable_runtime_safety'
                        ),
                        'recovering': self.recovering,
                        'latched': self.latched,
                    },
                    allow_nan=False,
                )
            )
            self.status_pub.publish(status)
            if reason != self.last_reason:
                self.last_reason = reason
                self.get_logger().info(reason)
            return
        if reason == self.last_reason:
            return
        self.last_reason = reason
        status = String()
        status.data = reason
        self.status_pub.publish(status)
        self.get_logger().info(reason)

    def tick(self):
        reason = self.health_reason()
        now = self.get_clock().now()
        # A brief receipt gap stops output immediately, but need not cancel a
        # healthy Nav2 goal. Escalate a continuous gap after 0.5 s; explicit lost,
        # invalid data and timestamp/TF failures still latch immediately.
        receipt_gap = reason in ('stale_odom_info', 'stale_odom_localization')
        if receipt_gap:
            if getattr(self, 'receipt_gap_since', None) is None:
                self.receipt_gap_since = now
        else:
            self.receipt_gap_since = None
        receipt_gap_persistent = (
            receipt_gap and (now - self.receipt_gap_since).nanoseconds / 1e9 >= 0.5
        )
        tracking_failure = reason in (
            'visual_odometry_lost',
            'waiting_odom_info',
            'stale_odom_info',
            'stale_tracking_stamp',
            'waiting_odom_localization',
            'stale_odom_localization',
            'invalid_odom_pose_or_covariance',
            'invalid_odometry_twist',
            'missing_tf',
            'invalid_tf_transform',
            'invalid_tf_map_odom',
            'invalid_tf_odom_base_link',
            'invalid_tf_map_base_link',
            'stale_tf_map_odom',
            'stale_tf_odom_base_link',
        )
        if (
            getattr(self, 'strict_tracking_safety', False)
            and tracking_failure
            and (not receipt_gap or receipt_gap_persistent)
            and self.ever_ready
            and self.goal_is_active()
            and not self.latched
        ):
            self.latched = self.tracking_stop_latched = True
            self.latch_reason = 'tracking loss: ' + reason
        if reason:
            if not getattr(self, 'coarse_hint_gate_enabled', False) or self.runtime_safety_reason:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = now  # Healthy runtime accumulates while verdict is pending.
            recoverable = reason in (
                'visual_odometry_lost',
                'waiting_odom_info',
                'stale_odom_info',
                'stale_tracking_stamp',
                'waiting_native_localization',
                'waiting_odom_localization',
                'invalid_odom_pose_or_covariance',
                'stale_odom_localization',
                'invalid_odometry_twist',
                'invalid_map_pose_or_covariance',
                'invalid_tf_transform',
                'missing_tf',
                'invalid_tf_map_odom',
                'invalid_tf_odom_base_link',
                'invalid_tf_map_base_link',
                'stale_tf_map_odom',
                'stale_tf_odom_base_link',
                'missing_coarse_hint_verified_ready',
                'stale_coarse_hint_verified_ready',
                'coarse_hint_verified_ready_false',
                'stopped_stale_controller_path',
                'stopped_invalid_controller_command',
                'stopped_stale_controller_command',
            ) or reason.startswith(('stale_lifecycle_', 'waiting_active_'))
            if self.ever_ready and (self.goal_source != 'nav2' or not recoverable):
                self.latched = True
                if not self.latch_reason:
                    self.latch_reason = reason
            if not self.recovering:
                self.recovery_started = now
                self.recovery_cause = reason
            self.recovering = True
            self.path_clear = False
            self.path_time = self.command_time = None
        elif self.stable_since is None:
            self.stable_since = now
        stable = (
            self.stable_since is not None
            and (now - self.stable_since).nanoseconds / 1e9
            >= self.get_parameter('stable_duration_s').value
        )
        self.runtime_stable = stable
        ready = not reason and stable and not self.latched
        self.final_navigation_ready = ready
        if ready and self.recovering:
            duration = (now - self.recovery_started).nanoseconds / 1e9
            self.report(f'recovery_completed: {duration:.2f}s; waiting_new_plan')
            self.recovering = False
            self.recovery_started = None
            self.resume_after = now
            self.path_clear = False
            self.path_time = self.command_time = None
        self.ever_ready |= ready
        status = Bool()
        status.data = ready
        self.ready_pub.publish(status)
        moving = ready and self.goal_is_active()
        # Native Nav2 owns the active path, collision checks and replanning.
        # Smac stamps a path BEFORE search: a 9-second search must not produce
        # an already-expired path, nor expire it during the next search. Goal,
        # map and health transitions invalidate path_clear independently.
        native_path = (
            getattr(self, 'native_localization_only', False) and self.goal_source == 'nav2'
        )
        path_ok = self.path_clear and (
            native_path or self.recent(self.path_time, self.get_parameter('path_timeout_s').value)
        )
        output = Twist()
        finite_command = self.command is not None and all(
            math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z)
        )
        if moving and path_ok and self.recent(self.command_time, 0.3):
            if finite_command:
                output = self.command
        self.publisher.publish(output)
        stop_status = Bool()
        stop_status.data = self.latched
        self.stop_latched_pub.publish(stop_status)
        if self.latched:
            suffix = (
                '; acknowledge_tracking_stop required after recovery'
                if getattr(self, 'tracking_stop_latched', False)
                else '; operator restart required'
            )
            self.report('stop_latched: ' + self.latch_reason + suffix)
        elif not ready:
            prefix = 'recovering_localization: ' if self.recovering else ''
            if getattr(self, 'coarse_hint_gate_enabled', False):
                self.report(
                    prefix
                    + (
                        self.runtime_safety_reason
                        or self.localization_ready_reason
                        or 'waiting_stable_runtime_safety'
                    )
                )
            else:
                self.report(prefix + (reason or 'waiting_stable_localization'))
        elif not moving:
            self.report(
                'ready_goal_' + self.goal_outcome
                if self.goal_outcome
                else 'ready_waiting_goal_or_completion'
            )
        elif not path_ok:
            self.report('stopped_waiting_valid_path')
        elif not self.recent(self.command_time, 0.3):
            self.report('stopped_stale_controller_command')
        elif not finite_command:
            self.report('stopped_invalid_controller_command')
        elif self.odom_rejection:
            self.report('driving_discarded_odom: ' + self.odom_rejection)
        else:
            self.report('driving')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can race taking a message against context shutdown. Do not
        # suppress a runtime failure while the ROS context is still active.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
