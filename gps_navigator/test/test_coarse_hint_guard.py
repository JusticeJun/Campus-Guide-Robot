"""The verifier owns place correctness; the guard retains runtime output safety."""

from copy import deepcopy
import json
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import pytest

from gps_navigator.localization_guard_node import LocalizationGuard
from test_guard_readiness import ros_time

pytest_plugins = ['test_guard_readiness']


def enable(runtime):
    guard, refresh, transforms, readiness, commands = runtime
    old_parameter = guard.get_parameter
    extra = {
        'coarse_hint_verified_ready_timeout_s': 2.5,
        'coarse_hint_runtime_odom_tf_timeout_s': 2.0,
    }
    guard.get_parameter = lambda k: (
        SimpleNamespace(value=extra[k]) if k in extra else old_parameter(k)
    )
    guard.coarse_hint_gate_enabled = True
    guard.coarse_hint_verified_ready = False
    guard.coarse_hint_verified_ready_time = None
    guard.runtime_safety_reason = guard.localization_ready_reason = ''
    guard.runtime_stable = guard.final_navigation_ready = False
    guard.goal_started = ros_time(10.0)
    guard.tick()  # Runtime stabilization starts even though verdict is missing.
    refresh(13.1)
    return guard, refresh, transforms, readiness, commands


def verified(runtime):
    guard, *_ = enable(runtime)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert runtime[3][-1] is True
    return runtime


def test_disabled_verdict_does_not_block_existing_guard(runtime):
    guard, refresh, _, readiness, commands = runtime
    guard.coarse_hint_gate_enabled = False
    guard.coarse_hint_verified_ready = False
    guard.coarse_hint_verified_ready_time = None
    guard.tick()
    refresh(13.1)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4
    refresh(100.0)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4


def test_enabled_false_and_missing_localization_verdict_block_healthy_runtime(runtime):
    guard, _, _, readiness, commands = enable(runtime)
    guard.tick()
    assert guard.health_reason() == 'missing_coarse_hint_verified_ready'
    assert not readiness[-1] and commands[-1] == 0.0
    guard.coarse_hint_ready_callback(Bool(data=False))
    assert guard.health_reason() == 'coarse_hint_verified_ready_false'
    assert not readiness[-1] and commands[-1] == 0.0
    assert guard.runtime_stable


def test_fresh_true_uses_accumulated_runtime_stability_without_new_global_wait(runtime):
    guard, _, _, readiness, _ = enable(runtime)
    guard.loc = PoseWithCovarianceStamped()  # Explicit bad global covariance is not a new verdict.
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert guard.health_reason() == '' and readiness[-1]
    assert not guard.recovering and not guard.latched


@pytest.mark.parametrize(
    'failure,reason',
    [
        ('odom_stale', 'stale_odom_localization'),
        ('odom_invalid', 'invalid_odom_pose_or_covariance'),
        ('map_tf_stale', 'stale_tf_map_odom'),
        ('odom_tf_stale', 'stale_tf_odom_base_link'),
        ('tf_missing', 'missing_tf'),
        ('lifecycle', 'waiting_active_controller_server'),
    ],
)
def test_verdict_cannot_override_runtime_failures(runtime, failure, reason):
    guard, refresh, transforms, readiness, commands = verified(runtime)
    if failure == 'odom_stale':
        guard.odom_time = ros_time(10.0)
    elif failure == 'odom_invalid':
        bad = deepcopy(guard.odom)
        bad.pose.pose.orientation.w = 0.0
        guard.odometry_callback(bad)  # Immediate stop even with cached fresh valid odom.
    elif failure == 'map_tf_stale':
        transforms['map', 'odom'].header.stamp = ros_time(10.0).to_msg()
    elif failure == 'odom_tf_stale':
        transforms['odom', 'base_link'].header.stamp = ros_time(10.0).to_msg()
    elif failure == 'tf_missing':
        transforms.pop(('map', 'base_link'))
    else:
        guard.states['controller_server'] = (2, ros_time(13.1))
    guard.tick()
    assert guard.health_reason() == reason
    assert not readiness[-1] and commands[-1] == 0.0


@pytest.mark.parametrize('stop', ['operator', 'obstacle', 'safety_latch'])
def test_verdict_cannot_clear_stop_latches(runtime, stop, monkeypatch):
    guard, refresh, _, readiness, commands = verified(runtime)
    if stop == 'operator':
        guard.stop_callback(Trigger.Request(), Trigger.Response())
    elif stop == 'obstacle':
        refresh(13.2)
        guard.route_map = SimpleNamespace(header=SimpleNamespace(stamp=ros_time(13.0).to_msg()))
        guard.footprint = []
        old = guard.get_parameter
        guard.get_parameter = lambda k: (
            SimpleNamespace(value=0.0) if k == 'footprint_padding' else old(k)
        )
        monkeypatch.setattr(
            'gps_navigator.localization_guard_node.path_is_clear', lambda *a: False
        )
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = ros_time(13.2).to_msg()
        guard.path_callback(plan)
    else:
        guard.latched = True
        guard.latch_reason = 'safety latch'
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert guard.latched and not readiness[-1] and commands[-1] == 0.0


def test_verdict_stale_and_clock_reset_fail_closed(runtime):
    guard, refresh, _, readiness, commands = verified(runtime)
    refresh(15.6)
    guard.tick()
    assert readiness[-1]  # Exactly at 2.5 s deadline.
    refresh(15.65)
    guard.tick()
    assert guard.health_reason() == 'stale_coarse_hint_verified_ready'
    assert not readiness[-1] and commands[-1] == 0.0
    refresh(12.0)
    guard.tick()
    assert not readiness[-1]


@pytest.mark.parametrize('cause', ['new_hint', 'relocalization', 'validation_failure'])
def test_false_verdict_revokes_immediately_preserves_nav2_goal_and_needs_new_plan(runtime, cause):
    guard, refresh, _, readiness, commands = verified(runtime)
    guard.path_clear = True
    guard.command_time = guard.path_time = ros_time(13.1)
    guard.coarse_hint_ready_callback(Bool(data=False))
    assert not readiness[-1] and commands[-1] == 0.0
    assert guard.active_goals == {b'goal'} and not guard.latched and guard.recovering
    assert guard.command_time is None and guard.path_time is None and not guard.path_clear
    refresh(13.2)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert readiness[-1]  # No localization confirmation wait after a new verifier verdict.
    assert guard.active_goals == {b'goal'} and commands[-1] == 0.0
    assert guard.resume_after == ros_time(13.2) and not guard.path_clear


def test_runtime_recovery_still_requires_existing_three_seconds(runtime):
    guard, refresh, _, readiness, commands = verified(runtime)
    guard.odom_time = ros_time(10.0)
    guard.tick()
    refresh(14.0)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert not readiness[-1]
    refresh(16.9)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert not readiness[-1]
    refresh(17.0)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert readiness[-1]
    assert commands[-1] == 0.0


@pytest.mark.parametrize('fault', ['stale_command', 'invalid_command', 'stale_path'])
def test_unhealthy_active_control_stream_withholds_ready(runtime, fault):
    guard, _, _, readiness, commands = verified(runtime)
    guard.path_clear = True
    guard.path_time = ros_time(13.1)
    guard.command_time = ros_time(13.1)
    if fault == 'stale_command':
        guard.command_time = ros_time(12.0)
    elif fault == 'invalid_command':
        guard.command.linear.x = float('nan')
    else:
        guard.path_time = ros_time(8.0)
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0
    assert 'controller_' in guard.runtime_safety_reason
    assert guard.recovering and not guard.latched and guard.active_goals == {b'goal'}


def test_control_stream_loss_can_replan_after_runtime_recovery_without_cached_output(runtime):
    guard, refresh, _, readiness, commands = verified(runtime)
    guard.path_clear = True
    guard.path_time = ros_time(13.1)
    guard.command_time = ros_time(12.0)
    guard.tick()
    assert not readiness[-1]
    refresh(13.2)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert not readiness[-1] and not guard.path_clear
    refresh(16.2)
    guard.coarse_hint_ready_callback(Bool(data=True))
    assert readiness[-1] and commands[-1] == 0.0 and guard.active_goals == {b'goal'}


def test_no_second_global_evidence_threshold_and_diagnostics_separate_verdicts(runtime):
    guard, _, _, readiness, _ = verified(runtime)
    guard.get_logger = lambda: SimpleNamespace(info=lambda text: None)
    guard.last_reason = ''
    statuses = []
    guard.status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
    guard.report = lambda reason: LocalizationGuard.report(guard, reason)
    guard.localization_callback(PoseWithCovarianceStamped())
    guard.rtab_info = None  # No match counts, loop closures or other confirmation source.
    guard.tick()
    assert readiness[-1]
    status = statuses[-1]
    assert status['localization_ready_source'] == 'coarse_hint_verified_ready'
    assert (
        status['localization_ready']
        and status['runtime_safety_ready']
        and status['final_navigation_ready']
    )
    guard.odom_time = ros_time(10.0)
    guard.tick()
    status = statuses[-1]
    assert (
        status['localization_ready']
        and not status['runtime_safety_ready']
        and not status['final_navigation_ready']
    )
    assert status['not_ready_reason'] == 'stale_odom_localization'
