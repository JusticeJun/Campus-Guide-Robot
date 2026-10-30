"""ExternalNav frames, timing, message encoding and independent command gating."""

import math
import time
from types import SimpleNamespace

from mavros.mavlink import convert_to_bytes
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as dialect
import pytest
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rover_navigation.external_navigation import (
    StreamHealth,
    convert_odometry,
    mavlink_packet,
    ros_mavlink,
)
from rover_navigation.external_navigation_node import ExternalNavigation
from test_localization_launch import expand, process
import test_nav2_integration as adapter_tests
import test_guard_readiness as guard_tests

# Explicit fixture exports also work when their defining test modules were
# already collected; registering those modules as late plugins does not.
hold_adapter = adapter_tests.hold_adapter
runtime = guard_tests.runtime
strict_runtime = guard_tests.strict_runtime


def odometry(t=10.0, yaw=0.0):
    """Construct a ROS body FLU velocity and continuous local pose."""
    msg = Odometry()
    msg.header.stamp = Time(seconds=t).to_msg()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.covariance = np.diag([0.01] * 6).ravel().tolist()
    msg.twist.covariance = np.diag([0.01] * 6).ravel().tolist()
    return msg


def test_local_axes_and_lever_arm_are_not_north_alignment():
    msg = odometry(yaw=math.pi / 2)
    msg.twist.twist.linear.x = 0.35
    msg.twist.twist.angular.z = 0.2
    e = convert_odometry(msg, [0.25, 0.0, 0.07])
    assert e.position == pytest.approx([0.0, -0.25, -0.07])
    assert e.velocity == pytest.approx([0.35, -0.05, 0.0])
    assert e.angular == pytest.approx([0.0, 0.0, -0.2])
    assert e.quaternion == pytest.approx([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])
    # Rotation-induced covariance and correlations survive the frame conversion.
    assert e.pose_covariance[6] >= 0.02
    assert any(abs(e.pose_covariance[i]) > 0 for i in (3, 4, 5, 8, 9, 10))


def test_roll_and_pitch_are_preserved_and_body_velocity_not_world_rotated():
    msg = odometry()
    msg.pose.pose.orientation.x = math.sin(0.1)
    msg.pose.pose.orientation.w = math.cos(0.1)
    msg.twist.twist.linear.y = 0.2
    e = convert_odometry(msg, [0.0, 0.0, 0.0])
    assert e.quaternion == pytest.approx([math.cos(0.1), math.sin(0.1), 0.0, 0.0])
    assert e.velocity == pytest.approx([0.0, -0.2, 0.0])


@pytest.mark.parametrize('fault', ['frame', 'child', 'nan', 'quat', 'cov', 'unknown', 'speed'])
def test_invalid_samples_rejected(fault):
    msg = odometry()
    if fault == 'frame':
        msg.header.frame_id = 'map'
    elif fault == 'child':
        msg.child_frame_id = 'camera_link'
    elif fault == 'nan':
        msg.twist.twist.angular.z = float('nan')
    elif fault == 'quat':
        msg.pose.pose.orientation.w = 0.0
    elif fault == 'cov':
        msg.pose.covariance[0] = -1.0
    elif fault == 'unknown':
        msg.twist.covariance[0] = 9999.0
    else:
        msg.twist.twist.linear.x = 2.0
    with pytest.raises(ValueError):
        convert_odometry(msg, [0.25, 0.0, 0.07])


def test_generated_wire_packet_roundtrip_crc_frames_quality_and_reset():
    codec = dialect.MAVLink(None, srcSystem=1, srcComponent=197)
    e = convert_odometry(odometry(), [0.25, 0.0, 0.07])
    packet = mavlink_packet(e, codec, 254)
    ros = ros_mavlink(packet)
    assert bytes(convert_to_bytes(ros)) == packet
    decoded = dialect.MAVLink(None).parse_char(packet)
    assert decoded.frame_id == 20 and decoded.child_frame_id == 12
    assert decoded.time_usec == 10000000
    assert decoded.reset_counter == 254 and decoded.quality == 1
    assert decoded.x == pytest.approx(0.25)
    assert decoded.z == pytest.approx(-0.07)
    assert codec.seq == 1


def test_actual_sample_rate_and_watchdog_not_replayed():
    health = StreamHealth()
    for i in range(33):
        t = 10.0 + i / 15
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t, t)
    assert health.ready(t)
    assert not health.ready(t + 0.61)
    with pytest.raises(ValueError, match='timestamp reset'):
        health.accept(health.last, t, t)
    with pytest.raises(ValueError, match='stale'):
        health.accept(convert_odometry(odometry(t + 1), [0.0, 0.0, 0.0]), t + 2, t + 2)
    health.reject('lost')
    assert not health.ready(t)


def test_sparse_or_reset_odometry_cannot_establish_readiness():
    health = StreamHealth()
    for i in range(20):
        t = 10.0 + i / 3
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t, t)
    assert not health.ready(t)
    assert health.fresh(t) and health.continuous(t)
    assert health.ready(t, minimum_hz=2.0)
    assert not health.ready(t + 0.61, minimum_hz=2.0)
    assert not health.ready(t, minimum_hz=0.0)
    msg = odometry(t + 0.1)
    msg.pose.pose.position.x = 5.0
    with pytest.raises(ValueError, match='discontinuity'):
        health.accept(convert_odometry(msg, [0.0, 0.0, 0.0]), t + 0.1, t + 0.1)


def test_low_rate_profile_tolerates_one_missing_frame_but_rejects_stale_data():
    health = StreamHealth(max_sample_age_s=0.8, max_gap_s=1.0)
    for dt in [0.0, 0.4, 0.8, 1.6, 2.0, 2.4]:
        t = 10.0 + dt
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t + 0.75, t)
    assert health.ready(t, minimum_hz=2.0)
    assert not health.ready(t + 1.01, minimum_hz=2.0)
    with pytest.raises(ValueError, match='stale'):
        health.accept(convert_odometry(odometry(t + 0.4), [0.0, 0.0, 0.0]), t + 1.21, t + 0.4)


def test_continuity_survives_rate_window_eviction_with_jitter():
    health = StreamHealth(max_sample_age_s=0.8, max_gap_s=1.0)
    # The first 0.7 s interval gets evicted from the 2.5 s rate window.
    # At 2.6 s the rate remains >2 Hz, but the window spans only 1.9 s.
    for dt in [0.0, 0.7, 1.0, 1.3, 1.6, 1.9, 2.2, 2.6, 2.9, 3.2]:
        t = 10.0 + dt
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t, t)
        if dt >= 2.2:
            assert health.ready(t, minimum_hz=2.0)
    health.reject('tracking lost')
    health.accept(convert_odometry(odometry(t + 0.1), [0.0, 0.0, 0.0]), t + 0.1, t + 0.1)
    assert not health.continuous(t + 0.1)


def test_one_second_age_profile_keeps_delayed_but_continuous_measurements():
    health = StreamHealth(max_sample_age_s=1.0, max_gap_s=1.0)
    for i in range(12):
        stamp = 10.0 + i / 3.0
        delay = 0.813 if i == 8 else 0.7
        health.accept(convert_odometry(odometry(stamp), [0.0, 0.0, 0.0]),
                      stamp + delay, stamp + delay)
        if i >= 6:
            assert health.ready(stamp + delay, minimum_hz=2.0)
    assert not health.ready(stamp + delay + 1.01, minimum_hz=2.0)
    with pytest.raises(ValueError, match='stale'):
        health.accept(convert_odometry(odometry(stamp + 0.3), [0.0, 0.0, 0.0]),
                      stamp + 1.31, stamp + 1.31)


@pytest.mark.parametrize('reception_gap', [False, True])
def test_real_gap_restarts_continuous_run(reception_gap):
    health = StreamHealth(max_sample_age_s=0.8, max_gap_s=1.0)
    for i in range(9):
        t = 10.0 + i * 0.3
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t, t)
    assert health.ready(t, minimum_hz=2.0)
    stamp = t + (0.3 if reception_gap else 1.1)
    arrival = t + 1.1
    health.accept(convert_odometry(odometry(stamp), [0.0, 0.0, 0.0]), stamp, arrival)
    assert health.fresh(arrival)
    assert not health.continuous(arrival)
    for i in range(1, 8):
        health.accept(convert_odometry(odometry(stamp + i * 0.3), [0.0, 0.0, 0.0]),
                      stamp + i * 0.3, arrival + i * 0.3)
    assert health.ready(arrival + 2.1, minimum_hz=2.0)


def test_two_second_field_gap_has_finite_cutoff():
    health = StreamHealth(max_sample_age_s=1.0, max_gap_s=2.0)
    for i in range(10):
        t = 10.0 + i / 3.0
        health.accept(convert_odometry(odometry(t), [0.0, 0.0, 0.0]), t, t)
    assert health.continuous(t + 1.7)
    assert not health.fresh(t + 2.001)
    health.accept(convert_odometry(odometry(t + 2.1), [0.0, 0.0, 0.0]), t + 2.1, t + 2.1)
    assert not health.continuous(t + 2.1)


def test_bridge_requires_disarmed_start_but_not_repeated_administrative_rpc():
    bridge = object.__new__(ExternalNavigation)
    bridge.state = State(connected=True, armed=True)
    bridge.state_at = bridge.params_at = 10.0
    bridge.params_ok, bridge.disarmed_seen, bridge.fault = True, False, ''
    assert not bridge.transport_ok(10.0)
    bridge.disarmed_seen = True
    assert bridge.transport_ok(10.0)
    bridge.state_at = 26.0
    assert bridge.transport_ok(26.0)  # RPC age alone is not a motion failure.
    bridge.params_at = 26.0
    bridge.fault = 'odom reset'
    assert not bridge.transport_ok(26.0)


def test_externalnav_adapter_loss_blocks_and_requests_hold(hold_adapter):
    node, now, calls, outputs = hold_adapter
    node.require_externalnav = True
    node.externalnav_ready, node.externalnav_at = True, now[0]
    node.fcu_state = State(connected=True, armed=True, mode='GUIDED')
    node.fcu_state_time = now[0]
    node.last_command, node.last_command_time_ns = (0.35, 0.1), int(now[0] * 1e9)
    node.publish_command()
    assert outputs[-1].velocity.x == 0.35
    # Brief heartbeat delivery jitter must preserve fresh upstream commands.
    for gap in (0.6, 1.1):
        now[0] += gap
        node.last_command_time_ns = int(now[0] * 1e9)
        node.publish_command()
        assert outputs[-1].velocity.x == 0.35
        assert not calls and not node.hold_pending
    # A continuous status loss beyond the grace period still stops motion.
    now[0] += 0.4
    node.last_command_time_ns = int(now[0] * 1e9)
    node.fcu_state_time = now[0]
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0
    assert calls[-1][0].custom_mode == 'HOLD'
    assert node.hold_pending


def test_guard_requires_independent_externalnav_heartbeat(runtime):
    guard, refresh, _, _, _ = runtime
    guard.require_externalnav = True
    guard.externalnav_ready, guard.externalnav_at = False, 0.0
    assert guard.runtime_health_reason() == 'external_navigation_not_ready'
    guard.externalnav_callback(Bool(data=True))
    assert guard.runtime_health_reason() != 'external_navigation_not_ready'
    guard.externalnav_at -= 1.7
    assert guard.runtime_health_reason() != 'external_navigation_not_ready'
    guard.externalnav_at -= 0.4
    assert guard.runtime_health_reason() == 'external_navigation_not_ready'


def test_default_launch_externalnav_and_double_gate(tmp_path):
    result, commands = expand(tmp_path, mode='drive')
    assert result == 0
    _, bridge = process(commands, 'external_navigation')
    assert bridge['transmit'] and bridge['mounting_reviewed']
    assert bridge['base_to_fcu_m'] == [0.25, 0.0, 0.07]
    assert process(commands, 'localization_guard')[1]['require_externalnav']
    assert process(commands, 'localization_guard')[1]['odometry_timeout_s'] == 2.0
    assert process(commands, 'pixhawk_command_adapter')[1]['require_externalnav']


def test_explicit_gps_launch_preserves_rollback_mode(tmp_path):
    result, commands = expand(tmp_path, mode='drive', fcu_estimation='gps')
    assert result == 0
    assert not any('external_navigation' == cmd[0].split('/')[-1] for cmd, _ in commands)
    assert not process(commands, 'pixhawk_command_adapter')[1]['require_externalnav']


def test_validation_transmits_estimates_without_vehicle_adapter(tmp_path):
    result, commands = expand(tmp_path, mode='validation')
    assert result == 0
    assert process(commands, 'external_navigation')[1]['transmit']
    assert not any('pixhawk_command_adapter' == cmd[0].split('/')[-1] for cmd, _ in commands)
    result, commands = expand(tmp_path, mode='validation', externalnav_transmit='false')
    assert result == 0
    assert not process(commands, 'external_navigation')[1]['transmit']


def test_externalnav_drive_refuses_preview_only(tmp_path):
    result, _ = expand(
        tmp_path, mode='drive', fcu_estimation='externalnav', externalnav_transmit='false'
    )
    assert result != 0


def test_externalnav_stop_can_be_acknowledged_without_reloading_map(strict_runtime):
    guard, update, _, readiness, commands = strict_runtime
    guard.require_externalnav = True
    guard.externalnav_callback(Bool(data=False))
    guard.tick()
    assert guard.tracking_stop_latched and commands[-1] == 0.0
    assert not guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    guard.active_goals = set()
    guard.externalnav_callback(Bool(data=True))
    update(14.0)
    guard.tick()
    update(17.1)
    guard.tick()
    assert guard.acknowledge_tracking_stop(None, Trigger.Response()).success
    assert not guard.latched and not guard.path_clear and guard.command is None
    update(21.0)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.0  # New goal still required.


def test_externalnav_wait_without_goal_does_not_permanently_latch(strict_runtime):
    guard, _, _, _, commands = strict_runtime
    guard.require_externalnav = True
    guard.active_goals = set()
    guard.externalnav_callback(Bool(data=False))
    guard.tick()
    assert not guard.latched and commands[-1] == 0.0


def test_guard_preserves_active_goal_during_bounded_externalnav_gap(strict_runtime):
    guard, _, _, readiness, commands = strict_runtime
    original_parameter = guard.get_parameter
    guard.get_parameter = lambda key: (
        SimpleNamespace(value=2.0) if key == 'odometry_timeout_s' else original_parameter(key)
    )
    guard.require_externalnav = True
    guard.externalnav_callback(Bool(data=True))
    # The established bridge can remain healthy while FCU estimation continues
    # through a 1.7 s measurement gap. Keep the live command and same goal.
    guard.externalnav_at -= 1.7  # The independent ready heartbeat can also lag.
    guard.odom_info_time = guard.odom_time = Time(
        seconds=11.4, clock_type=guard.get_clock().now().clock_type
    )
    goal = set(guard.active_goals)
    guard.tick()
    assert readiness[-1] and commands[-1] == 0.4
    assert not guard.latched and guard.active_goals == goal
    guard.externalnav_callback(Bool(data=False))
    guard.tick()
    assert not readiness[-1] and commands[-1] == 0.0 and guard.tracking_stop_latched


@pytest.mark.parametrize('fault', ['armed', 'moving', 'coordinates', 'stale_state', 'config'])
def test_bridge_recovery_does_not_override_real_faults(monkeypatch, fault):
    monkeypatch.setattr('rover_navigation.external_navigation_node.time.monotonic', lambda: 10.0)
    node = object.__new__(ExternalNavigation)
    node.fault, node.coordinate_fault = 'lost', False
    node.state, node.state_at = State(connected=True, armed=False), 10.0
    node.params_ok = True
    node.health = SimpleNamespace(
        continuous=lambda _: True, last=SimpleNamespace(velocity=np.zeros(3), angular=np.zeros(3))
    )
    if fault == 'armed':
        node.state.armed = True
    elif fault == 'moving':
        node.health.last.velocity[0] = 0.2
    elif fault == 'coordinates':
        node.coordinate_fault = True
    elif fault == 'stale_state':
        node.state_at = 0.0
    else:
        node.params_ok = False
    assert not node.recover(None, Trigger.Response()).success
    assert node.fault


def test_explicit_fcu_source_change_invalidates_verified_profile():
    from mavros_msgs.msg import ParamEvent
    from rcl_interfaces.msg import ParameterType, ParameterValue

    node = object.__new__(ExternalNavigation)
    node.params_ok = True
    node.on_parameter_event(
        ParamEvent(
            param_id='EK3_SRC1_VELXY',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=3),
        )
    )
    assert not node.params_ok and 'EK3_SRC1_VELXY' in node.params_reason


@pytest.mark.parametrize(
    'condition', ['disarmed', 'armed', 'stale', 'coordinates', 'previous_armed_fault']
)
def test_only_disarmed_transient_faults_can_automatically_recover(monkeypatch, condition):
    monkeypatch.setattr('rover_navigation.external_navigation_node.time.monotonic', lambda: 10.0)
    node = object.__new__(ExternalNavigation)
    node.sent, node.coordinate_fault, node.auto_recoverable = True, False, False
    node.state = State(connected=True, armed=condition == 'armed')
    node.state_at = 0.0 if condition == 'stale' else 10.0
    node.fault = 'previous motion failure' if condition == 'previous_armed_fault' else ''
    node.health = SimpleNamespace(reject=lambda _: None)
    node.ready_pub = SimpleNamespace(publish=lambda _: None)
    reason = 'odometry position discontinuity' if condition == 'coordinates' else 'stale odometry'
    node.fail(reason)
    assert node.fault
    assert node.auto_recoverable == (condition == 'disarmed')


@pytest.mark.parametrize('blocked', [None, 'armed', 'moving', 'stale', 'unstable', 'params'])
def test_coordinate_reinitialization_requires_disarmed_stable_input_and_new_epoch(
        tmp_path, blocked):
    node = object.__new__(ExternalNavigation)
    node.fault = 'odometry position discontinuity'
    node.coordinate_fault = True
    node.state = State(connected=True, armed=blocked == 'armed')
    node.state_at = time.monotonic() if blocked != 'stale' else 0
    node.params_ok = blocked != 'params'
    node.reset_counter = 25
    node.pending_odom = {1: 'old'}
    node.pending_info = {1: 'old'}
    rejected = []
    published = []
    node.health = SimpleNamespace(continuous=lambda _: blocked != 'unstable',
                                  last=SimpleNamespace(velocity=np.array(
                                      [.2 if blocked == 'moving' else 0., 0., 0.]),
                                      angular=np.zeros(3)),
                                  reject=rejected.append)
    node.ready_pub = SimpleNamespace(publish=lambda x: published.append(x.data))
    with (tmp_path/'reset').open('w+') as lease:
        lease.write('25')
        lease.flush()
        node.lease = lease
        response = node.reinitialize(None, Trigger.Response())
        assert response.success == (blocked is None)
        if blocked:
            assert node.fault and node.coordinate_fault and node.reset_counter == 25
        else:
            assert not node.fault and not node.coordinate_fault and node.reset_counter == 26
            assert (tmp_path/'reset').read_text() == '26'
            assert not node.pending_odom and not node.pending_info and rejected and published == [
                False]
