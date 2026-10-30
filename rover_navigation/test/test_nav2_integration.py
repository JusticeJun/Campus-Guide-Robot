import math
from types import SimpleNamespace

from geometry_msgs.msg import Twist
import pytest
from rclpy.time import Time

from rover_navigation.geo_utils import compass_heading_to_yaw, gps_to_local_xy
from rover_navigation.nav2_route_adapter_node import route_yaws
from rover_navigation.pixhawk_command_adapter_node import (
    bounded_motion,
    make_position_target,
    PixhawkCommandAdapter,
)


def test_gps_projection_uses_east_north_axes():
    east = gps_to_local_xy(35.0, 129.0001, 35.0, 129.0)
    north = gps_to_local_xy(35.0001, 129.0, 35.0, 129.0)
    assert east[0] > 0.0
    assert east[1] == pytest.approx(0.0)
    assert north[0] == pytest.approx(0.0)
    assert north[1] > 0.0


@pytest.mark.parametrize(
    ('heading', 'yaw'),
    [(0.0, math.pi / 2.0), (90.0, 0.0), (180.0, -math.pi / 2.0)],
)
def test_compass_heading_to_ros_yaw(heading, yaw):
    assert compass_heading_to_yaw(heading) == pytest.approx(yaw)


def test_route_orientations_follow_outgoing_segments():
    assert route_yaws([(0.0, 0.0), (2.0, 0.0), (2.0, 3.0)]) == pytest.approx(
        [0.0, math.pi / 2.0, math.pi / 2.0]
    )


def test_command_adapter_enforces_forward_speed_and_turning_radius():
    speed, yaw_rate = bounded_motion(0.8, 2.0, 0.6, 1.7)
    assert speed == pytest.approx(0.4)
    assert yaw_rate == pytest.approx(0.4 / 1.7)
    assert bounded_motion(-0.2, 0.5, 0.6, 1.7) == (0.0, 0.0)


@pytest.mark.parametrize('v', [.25, .3, .35, .4, .8])
@pytest.mark.parametrize('w', [-1.0, -.08, .0, .08, 1.0])
def test_reverse_enabled_has_symmetric_speed_curvature_limits(v, w):
    forward = bounded_motion(v, w, .4, 1.7, .4)
    reverse = bounded_motion(-v, -w, .4, 1.7, .4)
    assert reverse == pytest.approx((-forward[0], -forward[1]))
    target = make_position_target(*reverse)
    assert target.velocity.x < 0 and target.yaw_rate == reverse[1]
    assert not target.type_mask & target.IGNORE_YAW_RATE


def test_mavros_command_uses_body_forward_velocity_and_yaw_rate():
    command = make_position_target(0.4, -0.2)
    assert command.coordinate_frame == command.FRAME_BODY_NED
    assert command.velocity.x == pytest.approx(0.4)
    assert command.velocity.y == pytest.approx(0.0)
    assert command.yaw_rate == pytest.approx(-0.2)
    assert not command.type_mask & command.IGNORE_VX
    assert not command.type_mask & command.IGNORE_VY
    assert not command.type_mask & command.IGNORE_YAW_RATE


def test_adapter_watchdog_expires_without_replaying_last_motion():
    node = object.__new__(PixhawkCommandAdapter)
    now, outputs = [10.0], []
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node.get_parameter = lambda key: SimpleNamespace(
        value={
            'command_timeout_s': 0.5,
            'maximum_speed_m_s': 0.4,
            'maximum_reverse_speed_m_s': 0.0,
            'minimum_turning_radius_m': 1.7,
        }[key]
    )
    node.publisher = SimpleNamespace(publish=outputs.append)
    node.last_command = node.last_command_time_ns = None
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0
    command = Twist()
    command.linear.x = 0.4
    node.command_callback(command)
    node.publish_command()
    assert outputs[-1].velocity.x == 0.4
    for seconds in (10.6, 11.0, 20.0):
        now[0] = seconds
        node.publish_command()
        assert outputs[-1].velocity.x == outputs[-1].yaw_rate == 0.0


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_adapter_invalid_command_immediately_discards_previous_motion(invalid):
    node = object.__new__(PixhawkCommandAdapter)
    now, outputs = [10.0], []
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node.get_logger = lambda: SimpleNamespace(error=lambda text: None)
    node.get_parameter = lambda key: SimpleNamespace(
        value={
            'command_timeout_s': 0.5,
            'maximum_speed_m_s': 0.4,
            'maximum_reverse_speed_m_s': 0.0,
            'minimum_turning_radius_m': 1.7,
        }[key]
    )
    node.publisher = SimpleNamespace(publish=outputs.append)
    good = Twist()
    good.linear.x = 0.4
    node.command_callback(good)
    node.publish_command()
    assert outputs[-1].velocity.x == 0.4
    bad = Twist()
    bad.angular.z = invalid
    node.command_callback(bad)
    assert outputs[-1].velocity.x == outputs[-1].yaw_rate == 0.0
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0
    node.command_callback(good)
    now[0] = 9.0  # A backwards clock must not make the old command fresh forever.
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0


@pytest.fixture
def hold_adapter(monkeypatch):
    from rclpy.task import Future

    now, calls, outputs = [10.0], [], []
    monkeypatch.setattr(
        'rover_navigation.pixhawk_command_adapter_node.time.monotonic', lambda: now[0]
    )
    node = object.__new__(PixhawkCommandAdapter)
    node.hold_on_fault = True
    node.hold_pending = node.guard_stop = False
    node.fcu_state = None
    node.fcu_state_time = node.hold_request_time = -float('inf')
    node.hold_future = None
    node.last_command = node.last_command_time_ns = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node.get_logger = lambda: SimpleNamespace(error=lambda text: None)
    node.get_parameter = lambda key: SimpleNamespace(
        value={
            'command_timeout_s': 0.5,
            'maximum_speed_m_s': 0.4,
            'maximum_reverse_speed_m_s': 0.0,
            'minimum_turning_radius_m': 1.7,
        }[key]
    )
    node.publisher = SimpleNamespace(publish=outputs.append)

    def call(request):
        future = Future()
        calls.append((request, future))
        return future

    node.hold_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=call, remove_pending_request=lambda f: None
    )
    return node, now, calls, outputs


def test_fault_requires_hold_telemetry_and_guard_ack_before_accepting_new_motion(hold_adapter):
    from mavros_msgs.msg import State
    from std_msgs.msg import Bool

    node, now, calls, outputs = hold_adapter
    node.state_callback(State(connected=True, mode='GUIDED'))
    cmd = Twist()
    cmd.linear.x = 0.3
    node.command_callback(cmd)
    node.publish_command()
    assert outputs[-1].velocity.x == 0.3 and not calls
    node.stop_callback(Bool(data=True))
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0
    assert len(calls) == 1 and calls[0][0].custom_mode == 'HOLD'
    calls[0][1].set_result(SimpleNamespace(mode_sent=True))
    node.stop_callback(Bool(data=False))
    assert node.hold_pending  # Service reply alone is not mode telemetry.
    node.state_callback(State(connected=True, mode='HOLD'))
    assert not node.hold_pending and node.last_command is None
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0
    assert all(request.custom_mode == 'HOLD' for request, _ in calls)


def test_reverse_watchdog_zeros_and_requests_hold(hold_adapter):
    from mavros_msgs.msg import State
    node, now, calls, outputs = hold_adapter
    original = node.get_parameter
    node.get_parameter = lambda key: (SimpleNamespace(value=.4)
                                      if key == 'maximum_reverse_speed_m_s' else original(key))
    node.state_callback(State(connected=True, mode='GUIDED'))
    command = Twist()
    command.linear.x = -.35
    node.command_callback(command)
    node.publish_command()
    assert outputs[-1].velocity.x == -.35
    now[0] += .6
    node.publish_command()
    assert outputs[-1].velocity.x == 0.
    assert calls and calls[-1][0].custom_mode == 'HOLD'


def test_fault_hold_retries_unanswered_request_and_preserves_manual_override(hold_adapter):
    from mavros_msgs.msg import State
    from std_msgs.msg import Bool

    node, now, calls, _ = hold_adapter
    node.state_callback(State(connected=True, mode='GUIDED'))
    node.stop_callback(Bool(data=True))
    now[0] = 12.1
    node.state_callback(State(connected=True, mode='GUIDED'))
    assert len(calls) == 2
    now[0] = 15.0
    node.state_callback(State(connected=True, mode='MANUAL'))
    node.publish_command()
    assert len(calls) == 2 and node.hold_pending
    node.state_callback(State(connected=True, mode='HOLD'))
    assert node.hold_pending  # An active guard latch cannot be cleared by mode alone.
    node.stop_callback(Bool(data=False))
    assert not node.hold_pending


def test_motion_watchdog_requests_hold_but_startup_does_not(hold_adapter):
    from mavros_msgs.msg import State

    node, now, calls, outputs = hold_adapter
    node.state_callback(State(connected=True, mode='GUIDED'))
    node.publish_command()
    assert not calls
    cmd = Twist()
    cmd.linear.x = 0.3
    node.command_callback(cmd)
    now[0] = 10.6
    node.publish_command()
    assert outputs[-1].velocity.x == 0.0 and len(calls) == 1
