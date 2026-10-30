"""Live TF/odom/Nav2 establish and sustain readiness without RTAB pose events."""

from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.task import Future
from rtabmap_msgs.msg import Info, OdomInfo
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

from rover_navigation.localization_guard_node import LocalizationGuard


def ros_time(seconds):
    return Time(seconds=seconds, clock_type=ClockType.ROS_TIME)


@pytest.fixture
def strict_runtime(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    guard.strict_tracking_safety = True
    guard.tracking_max_age_s = 2.5
    guard.tracking_stop_latched = False
    guard.goal_transition_pending = False
    guard.odom_info = OdomInfo()

    def update(seconds):
        refresh(seconds)
        guard.odom_info.header.stamp = ros_time(seconds).to_msg()
        guard.odom_info_time = ros_time(seconds)

    update(10.0)
    guard.tick()
    update(13.1)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4
    return guard, update, transforms, readiness, commands


def test_explicit_tracking_loss_stops_in_callback_and_never_auto_resumes(strict_runtime):
    guard, update, _, readiness, commands = strict_runtime
    lost = OdomInfo(lost=True)
    lost.header.stamp = ros_time(13.1).to_msg()
    guard.odom_info_callback(lost)
    assert commands[-1] == 0.0 and readiness[-1] is False
    assert guard.latched and guard.tracking_stop_latched
    assert guard.latch_reason == 'tracking loss: visual_odometry_lost'
    guard.odom_info.lost = False
    for seconds in (13.2, 17.0, 100.0):
        update(seconds)
        guard.tick()
        assert commands[-1] == 0.0 and readiness[-1] is False


def test_strict_invalid_odom_has_no_one_second_grace(strict_runtime):
    guard, _, _, readiness, commands = strict_runtime
    bad = deepcopy(guard.odom)
    bad.pose.covariance[0] = 9999.0
    guard.odometry_callback(bad)
    assert commands[-1] == 0.0 and readiness[-1] is False
    assert guard.tracking_stop_latched


@pytest.mark.parametrize('failure', ['header_age', 'future_stamp', 'info_missing'])
def test_strict_stale_or_missing_tracking_blocks_output(strict_runtime, failure):
    guard, _, _, readiness, commands = strict_runtime
    if failure == 'header_age':
        guard.odom.header.stamp = ros_time(10.5).to_msg()
    elif failure == 'future_stamp':
        guard.odom_info.header.stamp = ros_time(14.0).to_msg()
    elif failure == 'info_missing':
        guard.odom_info = None
    guard.tick()
    assert commands[-1] == 0.0 and readiness[-1] is False
    assert guard.tracking_stop_latched


def test_receipt_gap_zeroes_immediately_and_persistent_gap_latches(strict_runtime):
    guard, update, _, readiness, commands = strict_runtime
    guard.odom_info_time = ros_time(12.0)
    guard.tick()
    assert commands[-1] == 0.0 and not readiness[-1]
    assert not guard.tracking_stop_latched
    # Switching between stale odom and stale info must not reset the timer.
    update(13.4)
    guard.odom_time = ros_time(12.0)
    guard.tick()
    assert commands[-1] == 0.0 and not guard.tracking_stop_latched
    update(13.61)
    guard.odom_info_time = ros_time(12.0)
    guard.tick()
    assert commands[-1] == 0.0 and guard.tracking_stop_latched


def test_explicit_loss_during_receipt_grace_latches_immediately(strict_runtime):
    guard, _, _, _, commands = strict_runtime
    guard.odom_info_time = ros_time(12.0)
    guard.tick()
    assert not guard.tracking_stop_latched
    guard.odom_info_callback(OdomInfo(lost=True))
    assert commands[-1] == 0.0 and guard.tracking_stop_latched


@pytest.mark.parametrize('age,reason', [(1.84, ''), (2.5, ''), (2.51, 'stale_tracking_stamp')])
def test_field_processing_delay_budget_keeps_a_finite_cutoff(strict_runtime, age, reason):
    guard, _, _, _, _ = strict_runtime
    guard.odom.header.stamp = ros_time(13.1 - age).to_msg()
    guard.odom_info.header.stamp = ros_time(13.1 - age).to_msg()
    assert guard.tracking_health_reason() == reason


def test_tracking_ack_requires_healthy_stationary_cancelled_goal_and_new_plan(strict_runtime):
    from std_srvs.srv import Trigger

    guard, update, _, readiness, commands = strict_runtime
    guard.odom_info.lost = True
    guard.tick()
    response = guard.acknowledge_tracking_stop(None, Trigger.Response())
    assert not response.success
    guard.odom_info.lost = False
    guard.active_goals = set()
    update(14.0)
    guard.tick()
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    update(17.1)
    guard.tick()
    guard.goal_transition_pending = True
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    guard.goal_transition_pending = False
    guard.cancel_request = Future()
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    guard.cancel_request.set_result(None)
    guard.odom.twist.twist.linear.x = 0.2
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    guard.odom.twist.twist.linear.x = 0.0
    response = guard.acknowledge_tracking_stop(None, Trigger.Response())
    assert response.success and not guard.latched and not guard.path_clear
    assert guard.command is None and guard.command_time is None
    assert commands[-1] == 0.0
    update(21.0)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.0  # No automatic old-goal restart.


def test_tracking_ack_never_clears_operator_stop(strict_runtime):
    from std_srvs.srv import Trigger

    guard, _, _, _, commands = strict_runtime
    guard.stop_callback(None, Trigger.Response())
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    assert guard.latched and commands[-1] == 0.0


@pytest.fixture
def runtime():
    guard = object.__new__(LocalizationGuard)
    now, readiness, commands = [10.0], [], []
    guard.get_clock = lambda: SimpleNamespace(now=lambda: ros_time(now[0]))
    parameters = {
        'localization_timeout_s': 2.0,
        'odometry_timeout_s': 1.0,
        'maximum_position_variance_m2': 0.25,
        'maximum_yaw_variance_rad2': 0.12,
        'stable_duration_s': 3.0,
        'path_timeout_s': 4.0,
    }
    guard.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    guard.loc, guard.odom = PoseWithCovarianceStamped(), Odometry()
    for msg, frame in ((guard.loc, 'map'), (guard.odom, 'odom')):
        msg.header.frame_id = frame
        msg.header.stamp = ros_time(1.0).to_msg()
        msg.pose.pose.orientation.w = 1.0
        for index in (0, 7, 35):
            msg.pose.covariance[index] = 0.01
    guard.odom.child_frame_id = 'base_link'
    guard.odom_rejection = ''
    guard.loc = None  # Most regressions run without any registration event.
    guard.rtab_info = Info()
    guard.rtab_info.header.frame_id = 'map'
    guard.invalid_tf = guard.tf_parent_conflict = False
    guard.invalid_tf_frames = set()
    guard.lifecycle_clients = {
        key: None for key in ('controller_server', 'planner_server', 'bt_navigator')
    }
    transforms = {}
    for parent, child in (
        ('map', 'odom'),
        ('odom', 'base_link'),
        ('base_link', 'camera_link'),
        ('map', 'base_link'),
    ):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.rotation.w = 1.0
        transforms[parent, child] = tf

    def lookup(parent, child, stamp):
        if (parent, child) not in transforms:
            raise TransformException('missing TF')
        return transforms[parent, child]

    guard.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    guard.stable_since = None
    guard.latched = guard.ever_ready = guard.recovering = False
    guard.resume_after = None
    guard.recovery_started = None
    guard.recovery_cause = guard.latch_reason = guard.goal_outcome = ''
    guard.goal_source = 'nav2'
    guard.active_goals = {b'goal'}
    guard.path_clear = True
    guard.command = Twist()
    guard.command.linear.x = 0.4
    guard.ready_pub = SimpleNamespace(publish=lambda msg: readiness.append(msg.data))
    guard.stop_latched_pub = SimpleNamespace(publish=lambda msg: None)
    guard.publisher = SimpleNamespace(publish=lambda msg: commands.append(msg.linear.x))
    guard.report = lambda reason: None

    def refresh(seconds, receive_odom=True, processing_delay=0.0):
        now[0] = seconds
        stamp = ros_time(seconds).to_msg()
        if receive_odom:
            guard.odom.header.stamp = ros_time(seconds - processing_delay).to_msg()
            guard.odom_time = ros_time(seconds)
        for (parent, _), tf in transforms.items():
            if parent != 'base_link':
                tf.header.stamp = stamp
        guard.states = {key: (3, ros_time(seconds)) for key in guard.lifecycle_clients}
        guard.path_time = guard.command_time = ros_time(seconds)

    refresh(10.0)
    guard.rtab_info.header.stamp = ros_time(1.0).to_msg()
    guard.rtab_info_time = ros_time(1.0)
    return guard, refresh, transforms, readiness, commands


@pytest.mark.parametrize('info_state', ['absent', 'old', 'wrong_frame', 'replayed'])
def test_single_old_registration_can_become_and_remain_ready(runtime, info_state):
    guard, refresh, _, readiness, commands = runtime
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = ros_time(1.0).to_msg()
    pose.pose = deepcopy(guard.odom.pose)
    guard.localization_callback(pose)
    if info_state == 'absent':
        guard.rtab_info = guard.rtab_info_time = None
    elif info_state == 'wrong_frame':
        guard.rtab_info.header.frame_id = 'odom'
    # One registration at t=1; no new match events at all, including during stability wait.
    assert guard.health_reason() == ''
    guard.tick()
    assert readiness[-1] is False
    for seconds in (13.1, 100.0, 1000.0):
        refresh(seconds)
        if info_state == 'replayed':
            guard.info_callback(guard.rtab_info)  # Fresh receipt, old source stamp.
        guard.tick()
        assert readiness[-1] is True and commands[-1] == 0.4
    assert guard.loc.header.stamp.sec == 1


@pytest.mark.parametrize(
    'failure,reason',
    [
        ('map_tf', 'stale_tf_map_odom'),
        ('missing_map_tf', 'missing_tf'),
        ('missing_odom_tf', 'missing_tf'),
        ('missing_chain', 'missing_tf'),
        ('odom', 'stale_odom_localization'),
        ('missing_odom', 'waiting_odom_localization'),
        ('odom_quaternion', 'stale_odom_localization'),
        ('odom_covariance', 'stale_odom_localization'),
        ('odom_high_covariance', 'stale_odom_localization'),
        ('odom_null', 'stale_odom_localization'),
        ('odom_position_nan', 'stale_odom_localization'),
        ('odom_twist_nan', 'stale_odom_localization'),
        ('odom_child_frame', 'invalid_odometry_child_frame'),
        ('map_covariance', 'invalid_map_pose_or_covariance'),
        ('tf_quaternion', 'invalid_tf_map_odom'),
        ('tf_nan', 'invalid_tf_odom_base_link'),
        ('future_tf', 'stale_tf_map_odom'),
        ('lifecycle', 'waiting_active_bt_navigator'),
        ('stale_lifecycle', 'stale_lifecycle_bt_navigator'),
    ],
)
def test_live_loss_revokes_ready_and_requires_stability(runtime, failure, reason):
    guard, refresh, transforms, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4
    refresh(16.0)
    healthy_loc, healthy_odom = deepcopy(guard.loc), deepcopy(guard.odom)
    healthy_transforms = deepcopy(transforms)
    old = ros_time(13.1).to_msg()
    if failure == 'map_tf':
        transforms['map', 'odom'].header.stamp = old
    elif failure == 'missing_map_tf':
        transforms.pop(('map', 'odom'))
    elif failure == 'missing_odom_tf':
        transforms.pop(('odom', 'base_link'))
    elif failure == 'missing_chain':
        transforms.pop(('map', 'base_link'))
    elif failure == 'odom':
        guard.odom_time = ros_time(13.1)  # Fresh header cannot mask a stopped stream.
    elif failure == 'missing_odom':
        guard.odom = None
    elif failure == 'odom_quaternion':
        guard.odom.pose.pose.orientation.w = 0.0
    elif failure == 'odom_covariance':
        guard.odom.pose.covariance[0] = float('nan')
    elif failure == 'odom_high_covariance':
        guard.odom.pose.covariance[0] = 9999.0
    elif failure == 'odom_null':
        guard.odom.pose.pose.orientation.w = 0.0
        guard.odom.pose.covariance = [9999.0] * 36
    elif failure == 'odom_position_nan':
        guard.odom.pose.pose.position.x = float('nan')
    elif failure == 'odom_twist_nan':
        guard.odom.twist.twist.linear.x = float('nan')
    elif failure == 'odom_child_frame':
        guard.odom.child_frame_id = 'camera_link'
    elif failure == 'map_covariance':
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose = deepcopy(guard.odom.pose)
        pose.pose.covariance[35] = 9999.0
        guard.localization_callback(pose)
    elif failure == 'tf_quaternion':
        transforms['map', 'odom'].transform.rotation.w = 0.0
    elif failure == 'tf_nan':
        transforms['odom', 'base_link'].transform.translation.x = float('nan')
    elif failure == 'future_tf':
        transforms['map', 'odom'].header.stamp = ros_time(17.0).to_msg()
    elif failure == 'stale_lifecycle':
        guard.states['bt_navigator'] = (3, ros_time(13.1))
    else:
        guard.states['bt_navigator'] = (2, ros_time(16.0))
    if failure.startswith('odom_') or failure == 'missing_odom':
        rejected = guard.odom
        if rejected is not None:
            guard.odom = deepcopy(healthy_odom)
            guard.odom_time = ros_time(13.1)
        guard.odometry_callback(rejected)  # Invalid receipt cannot extend valid-data age.
    assert guard.health_reason() == reason
    guard.tick()
    hard = failure == 'odom_child_frame'
    assert not readiness[-1] and commands[-1] == 0.0
    assert guard.latched == hard
    # Recovery needs a full stability window and a new plan/command.
    guard.loc, guard.odom = healthy_loc, healthy_odom
    transforms.clear()
    transforms.update(healthy_transforms)
    for seconds in (17.0, 21.0):
        refresh(seconds)
        guard.odometry_callback(guard.odom)
        assert guard.health_reason() == ''
        guard.tick()
        assert readiness[-1] == (seconds == 21.0 and not hard)
        assert commands[-1] == 0.0


def test_live_tf_without_any_rtab_event_establishes_and_sustains_readiness(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    guard.rtab_info = guard.rtab_info_time = None
    # Nonzero map alignment, continuously refreshed stamps, no pose events at all.
    transforms['map', 'odom'].transform.translation.x = -0.114
    transforms['map', 'odom'].transform.translation.y = -0.246
    for step in range(101):
        seconds = 10.0 + step * 0.1
        refresh(seconds)
        assert guard.loc is None and guard.health_reason() == ''
        guard.tick()
        assert readiness[-1] == (step >= 30)
        assert commands[-1] == (0.4 if step >= 30 else 0.0)


def test_stopped_map_broadcaster_cannot_complete_stabilization(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    edge = ('map', 'odom')
    cached_stamp = deepcopy(transforms[edge].header.stamp)
    for step in range(81):
        refresh(10.0 + step * 0.05)
        transforms[edge].header.stamp = cached_stamp
        guard.tick()
        assert not readiness[-1] and commands[-1] == 0.0
    assert guard.health_reason() == f'stale_tf_{edge[0]}_{edge[1]}'
    assert not guard.ever_ready


def test_invalid_tf_update_is_detected_even_if_tf2_keeps_previous_valid_pose(runtime):
    guard, _, _, _, _ = runtime
    bad = TransformStamped()
    bad.header.frame_id, bad.child_frame_id = 'map', 'odom'
    bad.transform.rotation.w = 0.0
    guard.tf_callback(TFMessage(transforms=[bad]))  # Zero quaternion: tf2 would drop it.
    assert guard.health_reason() == 'invalid_tf_transform'


def test_info_can_stop_arriving_after_ready_without_stopping_output(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.info_callback(guard.rtab_info)
    guard.tick()
    refresh(13.1)
    guard.tick()
    assert readiness[-1]
    for seconds in (16.0, 100.0):
        refresh(seconds)
        guard.tick()
        assert readiness[-1] and commands[-1] == 0.4 and not guard.latched


@pytest.mark.parametrize('processing_delay', [0.0, 0.35, 0.8])
def test_healthy_three_hz_with_jitter_and_processing_delay(runtime, processing_delay):
    guard, refresh, _, readiness, commands = runtime
    now = 10.0
    refresh(now, processing_delay=processing_delay)
    guard.tick()
    # Mean 0.3325 s (~3 Hz), including the observed 0.57 s maximum interval.
    for gap in (0.23, 0.27, 0.57, 0.26) * 5:
        receipt = now
        for elapsed in (0.05, gap - 0.001):
            refresh(receipt + elapsed, receive_odom=False)
            assert guard.health_reason() == ''
            guard.tick()
            if receipt + elapsed >= 13.0:
                assert readiness[-1] and commands[-1] == 0.4
        now += gap
        refresh(now, processing_delay=processing_delay)
        guard.tick()
        assert not guard.latched
    assert readiness[-1] and commands[-1] == 0.4


def test_odom_cessation_deadline_and_recovery(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    assert readiness[-1]
    # Keep TF, Nav2, plans and commands healthy while only odometry stops arriving.
    for age in (0.57, 0.99, 1.0):
        refresh(13.1 + age, receive_odom=False)
        guard.tick()
        assert readiness[-1] and commands[-1] == 0.4
    refresh(14.15, receive_odom=False)  # First 20 Hz check after the 1 s deadline.
    assert guard.health_reason() == 'stale_odom_localization'
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0 and not guard.latched
    for seconds in (14.2, 18.0):
        refresh(seconds)
        assert guard.health_reason() == ''
        guard.tick()
        assert readiness[-1] == (seconds == 18.0)
        assert commands[-1] == 0.0


def test_odom_without_receipt_evidence_is_not_fresh(runtime):
    guard, _, _, readiness, commands = runtime
    guard.odom_time = None
    assert guard.health_reason() == 'stale_odom_localization'
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0


def test_slow_image_stamped_odom_tf_and_static_camera_remain_ready(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    guard.tf_buffer = Buffer()
    camera = transforms['base_link', 'camera_link']
    camera.header.stamp = ros_time(1.0).to_msg()
    guard.tf_buffer.set_transform_static(camera, 'static_camera')
    last_tf = 0.0
    max_age = 0.0
    for step in range(121):
        seconds = 10.0 + step * 0.05
        refresh(seconds, receive_odom=step % 7 == 0, processing_delay=0.35)
        # Stable map correction is still broadcast by RTAB's independent thread.
        map_tf = transforms['map', 'odom']
        map_tf.header.stamp = ros_time(seconds + 0.1).to_msg()
        guard.tf_buffer.set_transform(map_tf, 'rtabmap')
        # Exercise TF delivery slower than odometry, with image processing delay.
        if step % 22 == 0:
            last_tf = seconds - 0.35
            odom_tf = transforms['odom', 'base_link']
            odom_tf.header.stamp = ros_time(last_tf).to_msg()
            guard.tf_buffer.set_transform(odom_tf, 'rgbd_odometry')
        max_age = max(max_age, seconds - last_tf)
        if not guard.recovering:
            guard.path_clear = True  # Simulate new plans after initial TF warmup.
        guard.tick()
        if seconds >= 14.2:  # Initial common-history warmup plus stabilization.
            assert readiness[-1] and commands[-1] == 0.4
        assert not guard.latched
    assert max_age > 1.0 and guard.health_reason() == ''
    # TF is actually unavailable after clearing dynamic history; static remains.
    guard.tf_buffer.clear()
    guard.tick()
    assert guard.health_reason() == 'missing_tf'
    assert not readiness[-1] and commands[-1] == 0.0 and not guard.latched


def test_individually_available_edges_without_common_time_are_not_ready(runtime):
    guard, _, transforms, readiness, commands = runtime
    guard.tf_buffer = Buffer()
    guard.tf_buffer.set_transform_static(transforms['base_link', 'camera_link'], 'camera')
    transforms['odom', 'base_link'].header.stamp = ros_time(8.0).to_msg()
    for edge in (('map', 'odom'), ('odom', 'base_link')):
        guard.tf_buffer.set_transform(transforms[edge], 'test')
        assert guard.tf_buffer.can_transform(*edge, Time())
    assert guard.health_reason() == 'missing_tf'
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0


def test_static_camera_validity_and_parent_conflict_are_independent(runtime):
    guard, _, transforms, _, _ = runtime
    transforms['base_link', 'camera_link'].transform.rotation.w = 0.0
    assert guard.health_reason() == 'invalid_tf_base_link_camera_link'
    transforms['base_link', 'camera_link'].transform.rotation.w = 1.0
    conflict = deepcopy(transforms['odom', 'base_link'])
    conflict.header.frame_id = 'map'
    guard.tf_callback(TFMessage(transforms=[conflict]))
    assert guard.health_reason() == 'conflicting_tf_parent'


def test_invalid_tf_requires_good_update_for_the_same_edge(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    bad = deepcopy(transforms['odom', 'base_link'])
    bad.transform.rotation.w = 0.0
    guard.tf_callback(TFMessage(transforms=[bad]))
    assert not readiness[-1] and commands[-1] == 0.0
    guard.tf_callback(TFMessage(transforms=[transforms['map', 'odom']]))
    assert guard.invalid_tf
    guard.tf_callback(TFMessage(transforms=[transforms['odom', 'base_link']]))
    assert not guard.invalid_tf and not guard.latched


def test_timed_out_lifecycle_request_is_retried_and_late_reply_ignored(runtime):
    guard, refresh, _, _, _ = runtime
    name = 'bt_navigator'
    expired, fresh = Future(), Future()
    calls = []
    guard.lifecycle_clients = {
        name: SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: calls.append(request) or fresh,
        )
    }
    guard.requests = {name: expired}
    guard.request_times = {name: ros_time(10.0)}
    refresh(11.6)
    guard.poll_lifecycle()
    assert expired.cancelled() and len(calls) == 1
    guard.lifecycle_result(name, expired)
    assert guard.requests[name] is fresh
    fresh.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3)))
    assert name not in guard.requests and guard.states[name][0] == 3


@pytest.mark.parametrize('failure', ['quaternion', 'covariance', 'sentinel', 'position', 'twist'])
def test_isolated_invalid_sample_is_discarded_without_refreshing_valid_age(runtime, failure):
    guard, refresh, _, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    accepted, received = deepcopy(guard.odom), guard.odom_time
    bad = deepcopy(accepted)
    if failure == 'quaternion':
        bad.pose.pose.orientation.w = 0.0
    elif failure == 'covariance':
        bad.pose.covariance[0] = float('nan')
    elif failure == 'sentinel':
        bad.pose.covariance = [9999.0] * 36
    elif failure == 'position':
        bad.pose.pose.position.x = float('inf')
    else:
        bad.twist.twist.linear.x = float('nan')
    refresh(13.4, receive_odom=False)
    guard.odometry_callback(bad)
    assert guard.odom == accepted and guard.odom_time == received
    assert readiness[-1] and commands[-1] == 0.4 and not guard.recovering
    refresh(13.8, receive_odom=False)
    guard.odometry_callback(deepcopy(accepted))
    guard.tick()
    assert guard.odom_time == ros_time(13.8) and not guard.odom_rejection
    assert readiness[-1] and commands[-1] == 0.4 and not guard.latched


def test_invalid_stream_expires_at_original_valid_receipt_then_recovers(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    healthy = deepcopy(guard.odom)
    bad = deepcopy(healthy)
    bad.pose.pose.orientation.w = 0.0
    for age in (0.3, 0.6, 0.99, 1.0):
        refresh(13.1 + age, receive_odom=False)
        guard.odometry_callback(bad)
        assert guard.odom_time == ros_time(13.1)
        assert readiness[-1] and commands[-1] == 0.4
    for seconds in (14.15, 34.0, 1000.0):
        refresh(seconds, receive_odom=False)
        guard.odometry_callback(bad)
        assert not readiness[-1] and commands[-1] == 0.0 and not guard.latched
        assert guard.health_reason() == 'stale_odom_localization'
        assert guard.command_time is None and not guard.path_clear
    refresh(1001.0, receive_odom=False)
    guard.odometry_callback(healthy)
    guard.tick()
    for seconds in (1001.5, 1002.0, 1002.5, 1003.0, 1003.99, 1004.0):
        refresh(seconds)
        guard.tick()
        assert readiness[-1] == (seconds == 1004.0)
        assert commands[-1] == 0.0  # Release still needs a new plan and command.
    assert not guard.latched


def test_field_invalid_bursts_use_valid_age_without_a_new_count_threshold(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    healthy = deepcopy(guard.odom)
    bad = deepcopy(healthy)
    bad.pose.covariance[0] = 9999.0
    start = 13.1
    # Completed bursts measured in the existing 0929 bag. Includes two invalids.
    for invalid_offsets, good_offset in (((0.3, 0.6), 0.915), ((0.4,), 0.957), ((0.3,), 0.696)):
        for offset in invalid_offsets:
            refresh(start + offset, receive_odom=False)
            guard.odometry_callback(bad)
            assert readiness[-1] and commands[-1] == 0.4
        start += good_offset
        refresh(start, receive_odom=False)
        guard.odometry_callback(deepcopy(healthy))
        guard.tick()
        assert readiness[-1] and not guard.recovering and not guard.latched


@pytest.mark.parametrize('other_failure', ['tf', 'localization', 'command'])
def test_cached_valid_odom_cannot_mask_other_safety_failures(runtime, other_failure):
    guard, refresh, transforms, readiness, commands = runtime
    guard.tick()
    refresh(13.1)
    guard.tick()
    bad = deepcopy(guard.odom)
    bad.pose.pose.orientation.w = 0.0
    refresh(13.4, receive_odom=False)
    if other_failure == 'tf':
        transforms.pop(('map', 'base_link'))
    elif other_failure == 'localization':
        guard.loc = PoseWithCovarianceStamped()  # Explicit bad map estimate still stops.
    else:
        guard.command_time = ros_time(13.0)
    guard.odometry_callback(bad)
    assert commands[-1] == 0.0
    if other_failure != 'command':
        assert not readiness[-1]


def test_invalid_samples_before_first_valid_odom_cannot_establish_ready(runtime):
    guard, refresh, _, readiness, commands = runtime
    bad = deepcopy(guard.odom)
    bad.pose.pose.orientation.w = 0.0
    guard.odom = guard.odom_time = None
    for seconds in (10.0, 13.0, 100.0):
        refresh(seconds, receive_odom=False)
        guard.odometry_callback(bad)
        assert guard.odom is None and guard.odom_time is None
        assert not readiness[-1] and commands[-1] == 0.0 and not guard.latched


def test_native_mode_waits_for_registration_even_with_live_tf(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.native_localization_only = True
    guard.native_localization_initialized = False
    candidate = Info()
    candidate.stats_keys = ['Loop/Highest_hypothesis_id/']
    candidate.stats_values = [464.0]
    guard.info_callback(candidate)
    for seconds in (10.0, 13.0, 100.0):
        refresh(seconds)
        guard.tick()
        assert guard.health_reason() == 'waiting_native_localization'
        assert not readiness[-1] and commands[-1] == 0.0
    accepted = Info(loop_closure_id=464)
    guard.info_callback(accepted)
    guard.tick()
    refresh(103.1)
    guard.tick()
    assert readiness[-1] and not guard.latched


@pytest.mark.parametrize('name', ['bt_navigator', 'planner_server', 'controller_server'])
def test_nav2_pause_resumes_without_latching_and_requires_fresh_control(runtime, name):
    guard, refresh, _, readiness, commands = runtime
    guard.native_localization_only = True
    guard.native_localization_initialized = True
    guard.tick()
    refresh(13.1)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4
    active_goals = guard.active_goals.copy()
    refresh(16.0)
    guard.states[name] = (2, ros_time(16.0))
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0
    assert guard.recovering and not guard.latched
    assert guard.active_goals == active_goals
    for seconds in (17.0, 21.0):
        refresh(seconds)
        guard.tick()
    assert readiness[-1] and not guard.latched
    assert commands[-1] == 0.0 and not guard.path_clear
    assert guard.command_time is None  # The pre-pause command cannot resume motion.


@pytest.mark.parametrize('source', ['loop', 'proximity', 'landmark', 'pose'])
def test_native_registration_needs_no_continuous_match_or_second_covariance_gate(runtime, source):
    guard, refresh, _, readiness, _ = runtime
    guard.native_localization_only = True
    guard.native_localization_initialized = False
    if source == 'pose':
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose = deepcopy(guard.odom.pose)
        # Native RTAB accepted this registration; the guard does not re-score it.
        pose.pose.covariance[0] = 5.0
        guard.localization_callback(pose)
    else:
        info = Info()
        setattr(
            info,
            {
                'loop': 'loop_closure_id',
                'proximity': 'proximity_detection_id',
                'landmark': 'landmark_id',
            }[source],
            464,
        )
        guard.info_callback(info)
    assert guard.health_reason() == ''
    guard.tick()
    for seconds in (13.1, 100.0, 1000.0):
        refresh(seconds)
        guard.info_callback(Info())  # Tracking need not recognize a new place each frame.
        guard.tick()
        assert readiness[-1]
    refresh(1002.0, receive_odom=False)
    guard.tick()
    assert not readiness[-1] and guard.health_reason() == 'stale_odom_localization'
