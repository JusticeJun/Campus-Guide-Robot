"""Map-coordinate navigation needs neither GPS metadata nor a route heartbeat."""

from pathlib import Path as FilePath
from types import SimpleNamespace

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
import yaml

from gps_navigator.localization_guard_node import LocalizationGuard
from gps_navigator.map_session import select_session


@pytest.fixture
def map_directory(tmp_path):
    (tmp_path / 'rtabmap.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    (tmp_path / 'rtabmap.db').write_bytes(b'test only')
    (tmp_path / 'rtabmap.yaml').write_text(
        yaml.safe_dump(
            {
                'image': 'rtabmap.pgm',
                'origin': [0.0, 0.0, 0.0],
                'resolution': 0.05,
                'negate': 0,
                'occupied_thresh': 0.5,
                'free_thresh': 0.196,
            }
        )
    )
    return tmp_path


def test_map_directory_drive_has_no_gps_or_commissioning_dependency(map_directory):
    profile, params, runtime, _ = select_session('drive', map_dir=str(map_directory))
    assert profile['database'] == str(map_directory / 'rtabmap.db')
    assert profile['map'] == str(map_directory / 'rtabmap.yaml')
    assert 'alignment' not in profile and 'waypoints_file' not in profile
    assert FilePath(params).is_file() and FilePath(runtime).is_file()
    assert profile['base_to_camera'] == [0.3001, 0.0175, 0.2032, 0.0, 0.0, 0.0]


def test_explicit_artifacts_and_session_config_precedence(map_directory):
    local = map_directory / 'nav2.yaml'
    local.write_text('local: true\n')
    runtime = map_directory / 'localization.yaml'
    runtime.write_text('rtabmap: {}\n')
    _, params, localization, _ = select_session('validation', map_dir=str(map_directory))
    assert params == str(local) and localization == str(runtime)
    override = map_directory / 'custom.yaml'
    override.write_text('explicit: true\n')
    _, params, _, _ = select_session(
        'validation', map_dir=str(map_directory), params_file='custom.yaml'
    )
    assert params == str(override)
    profile, _, _, _ = select_session(
        'drive',
        occupancy=str(map_directory / 'rtabmap.yaml'),
        database=str(map_directory / 'rtabmap.db'),
    )
    assert profile['database'] == str(map_directory / 'rtabmap.db')


def test_map_selection_rejects_missing_and_ambiguous_artifacts(map_directory):
    with pytest.raises(ValueError, match='Missing navigation artifact'):
        select_session('drive', map_dir=str(map_directory), database='missing.db')
    (map_directory / 'map.db').write_bytes(b'another DB')
    with pytest.raises(ValueError, match='explicit artifact'):
        select_session('drive', map_dir=str(map_directory))
    with pytest.raises(ValueError, match='not both'):
        select_session('drive', map_dir=str(map_directory), profile='profile.yaml')


@pytest.fixture
def guard():
    node = object.__new__(LocalizationGuard)
    now, outputs, health = [10.0], [], ['']
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    )
    node.get_parameter = lambda key: SimpleNamespace(
        value={'stable_duration_s': 3.0, 'path_timeout_s': 4.0, 'footprint_padding': 0.0}[key]
    )
    node.publisher = SimpleNamespace(publish=lambda msg: outputs.append(msg.linear.x))
    node.ready_pub = SimpleNamespace(publish=lambda msg: None)
    node.stop_latched_pub = SimpleNamespace(publish=lambda msg: None)
    node.report = lambda text: None
    node.health_reason = lambda: health[0]
    node.goal_source = 'nav2'
    node.recovering = False
    node.odom_rejection = ''
    node.resume_after = None
    node.recovery_started = None
    node.recovery_cause = node.latch_reason = node.goal_outcome = ''
    node.tracked_goal_id = None
    node.active_goals = set()
    node.goal_started = None
    node.route_time = None
    node.command_time = node.path_time = None
    node.path_clear = node.ever_ready = node.latched = False
    node.stable_since = Time(clock_type=ClockType.ROS_TIME, seconds=6.0)
    node.footprint = [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]]
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1.0
    grid.info.width = grid.info.height = 20
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    node.route_map = grid
    node.command = Twist()
    node.command.linear.x = 0.4
    return node, now, outputs, health


def status(state, identifier=1):
    msg = GoalStatus()
    msg.goal_info.goal_id.uuid = [identifier] * 16
    msg.status = state
    return GoalStatusArray(status_list=[msg])


def send_plan(node, seconds):
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = Time(clock_type=ClockType.ROS_TIME, seconds=seconds).to_msg()
    pose = PoseStamped()
    pose.pose.position.x = pose.pose.position.y = 5.0
    pose.pose.orientation.w = 1.0
    msg.poses = [pose]
    node.path_callback(msg)


def test_native_slow_plan_survives_search_and_replanning_but_not_goal_change(guard):
    node, now, outputs, _ = guard
    node.native_localization_only = True
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    # Smac starts at t=10 and publishes at t=19. No output before completion.
    now[0] = 19.0
    node.tick()
    assert outputs[-1] == 0.0
    send_plan(node, 10.0)
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4
    now[0] = 28.0  # The controller follows the same path while replanning.
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4
    now[0] += 0.31
    node.tick()
    assert outputs[-1] == 0.0  # Controller receipt watchdog is unchanged.
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING, identifier=2))
    send_plan(node, 10.0)  # A delayed path from the previous goal cannot restart.
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.0 and not node.path_clear


def test_brief_receipt_gap_retains_goal_but_requires_stability_and_new_plan(guard):
    node, now, outputs, health = guard
    node.native_localization_only = node.strict_tracking_safety = True
    node.tracking_stop_latched = False
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, 10.0)
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4
    health[0] = 'stale_odom_localization'
    now[0] = 11.0
    node.tick()
    assert outputs[-1] == 0.0 and not node.latched
    health[0] = ''
    now[0] = 11.15
    node.tick()
    assert outputs[-1] == 0.0 and node.active_goals
    now[0] = 14.2
    node.tick()
    assert outputs[-1] == 0.0 and not node.latched
    send_plan(node, 10.0)  # Pre-recovery path is rejected.
    node.tick()
    assert outputs[-1] == 0.0
    now[0] = 23.3
    send_plan(node, 14.3)
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4 and not node.tracking_stop_latched


@pytest.mark.parametrize(
    'terminal',
    [
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELING,
    ],
)
def test_action_motion_and_terminal_stop_without_route_heartbeat(guard, terminal):
    node, now, outputs, _ = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    node.tick()
    assert outputs[-1] == 0.4 and node.route_time is None
    node.goal_status_callback(status(terminal))
    node.tick()
    assert outputs[-1] == 0.0 and not node.path_clear


def test_goal_replacement_requires_new_path_and_command(guard):
    node, now, outputs, _ = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    node.tick()
    assert outputs[-1] == 0.4
    now[0] += 0.1
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING, 2))
    send_plan(node, 10.0)  # Delayed old plan cannot enable the replacement goal.
    node.tick()
    assert outputs[-1] == 0.0 and not node.path_clear
    send_plan(node, now[0])
    node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    node.tick()
    assert outputs[-1] == 0.4


def test_normal_nav2_path_uses_planner_collision_check_without_second_raster_gate(guard):
    node, now, outputs, _ = guard
    node.native_localization_only = True
    node.route_map.data = [-1] * 400
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_callback(node.command)
    node.tick()
    assert node.path_clear and not node.latched and outputs[-1] == 0.4


def test_first_plan_uses_goal_acceptance_time_not_status_receipt_time(guard):
    node, now, _, _ = guard
    action = status(GoalStatus.STATUS_EXECUTING)
    action.status_list[0].goal_info.stamp = Time(
        clock_type=ClockType.ROS_TIME, seconds=now[0] - 0.5
    ).to_msg()
    node.goal_status_callback(action)
    send_plan(node, now[0] - 0.2)  # Created after acceptance, before status callback.
    assert node.path_clear


@pytest.mark.parametrize('failure', ['frame', 'empty', 'position', 'rotation', 'pose_frame'])
def test_normal_nav2_path_still_rejects_invalid_geometry(guard, failure):
    node, now, outputs, _ = guard
    node.native_localization_only = True
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = Time(clock_type=ClockType.ROS_TIME, seconds=now[0]).to_msg()
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    msg.poses = [pose]
    if failure == 'frame':
        msg.header.frame_id = 'odom'
    elif failure == 'empty':
        msg.poses = []
    elif failure == 'position':
        pose.pose.position.x = float('nan')
    elif failure == 'rotation':
        pose.pose.orientation.w = 0.0
    else:
        pose.header.frame_id = 'odom'
    node.path_callback(msg)
    assert node.latched and not node.path_clear and outputs[-1] == 0.0


@pytest.mark.parametrize('failure', ['localization', 'command', 'path', 'obstacle'])
def test_action_guard_stops_on_failures(guard, failure):
    node, now, outputs, health = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    node.tick()
    assert outputs[-1] == 0.4
    if failure == 'localization':
        health[0] = 'stale_tf_map_odom'
    elif failure == 'command':
        now[0] += 0.31
    elif failure == 'path':
        now[0] += 4.1
        node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
    else:
        node.route_map.data[5 * 20 + 5] = 100
        send_plan(node, now[0])
    node.tick()
    assert outputs[-1] == 0.0
    if failure == 'obstacle':
        assert node.latched
        health[0] = ''
        now[0] += 4.0
        node.route_map.data[5 * 20 + 5] = 0
        send_plan(node, now[0])
        node.command_time = Time(clock_type=ClockType.ROS_TIME, seconds=now[0])
        node.tick()
        assert outputs[-1] == 0.0


def test_operator_stop_cancels_common_action_and_stays_latched(guard):
    node, _, outputs, _ = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    response = node.stop_callback(None, SimpleNamespace())
    assert response.success and node.latched and outputs[-1] == 0.0
    requests = []
    node.cancel_request = None
    node.cancel_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda request: requests.append(request)
    )
    node.lifecycle_clients = {}
    node.poll_lifecycle()
    assert len(requests) == 1
    assert list(requests[0].goal_info.goal_id.uuid) == [0] * 16


def test_structural_failure_cancels_and_rejects_old_and_new_goals_after_recovery(guard):
    node, now, outputs, health = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4
    health[0] = 'conflicting_tf_parent'
    node.tick()
    requests = []
    node.cancel_request = None
    node.cancel_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda request: requests.append(request)
    )
    node.lifecycle_clients = {}
    node.poll_lifecycle()
    assert len(requests) == 1 and node.latched and outputs[-1] == 0.0
    health[0] = ''
    for identifier in (1, 2):
        now[0] += 4.0
        node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING, identifier))
        send_plan(node, now[0])
        node.command_callback(node.command)
        node.tick()
        node.poll_lifecycle()
        assert node.latched and outputs[-1] == 0.0
    assert len(requests) == 3


def test_prolonged_loss_preserves_goal_but_requires_post_recovery_plan_and_command(guard):
    node, now, outputs, health = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    send_plan(node, now[0])
    node.command_callback(node.command)
    node.tick()
    original = node.active_goals.copy()
    assert outputs[-1] == 0.4
    health[0] = 'stale_odom_localization'
    node.tick()
    requests = []
    node.lifecycle_clients = {}
    node.cancel_client = SimpleNamespace(call_async=lambda request: requests.append(request))
    node.poll_lifecycle()
    assert not requests and not node.latched and outputs[-1] == 0.0
    for seconds in (11.0, 31.0, 1000.0):  # Continuous zero while health remains invalid.
        now[0] = seconds
        send_plan(node, seconds)
        node.command_callback(node.command)
        node.tick()
        node.poll_lifecycle()
        assert not requests and not node.latched
        assert outputs[-1] == 0.0 and node.active_goals == original
    health[0] = ''
    now[0] = 1001.0
    node.tick()
    now[0] = 1003.99
    node.tick()
    assert node.recovering and outputs[-1] == 0.0
    now[0] = 1004.0
    node.tick()
    assert not node.recovering and node.active_goals == original
    node.command_callback(node.command)
    send_plan(node, 12.0)  # Delayed old plan must not reopen output.
    node.tick()
    assert outputs[-1] == 0.0 and not node.path_clear
    now[0] = 1004.1
    send_plan(node, now[0])
    node.tick()
    assert outputs[-1] == 0.0  # Plan alone cannot replay the cached command.
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.4 and node.active_goals == original


@pytest.mark.parametrize('terminal', [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED])
def test_goal_ended_during_recovery_is_never_resurrected(guard, terminal):
    node, now, outputs, health = guard
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    node.tick()
    health[0] = 'missing_tf'
    node.tick()
    node.goal_status_callback(status(terminal))
    health[0] = ''
    now[0] = 11.0
    node.tick()
    now[0] = 14.0
    node.tick()
    now[0] = 14.1
    send_plan(node, now[0])
    node.command_callback(node.command)
    node.tick()
    assert outputs[-1] == 0.0 and not node.active_goals and not node.path_clear


def test_canceling_then_terminal_keeps_action_outcome_visible(guard):
    node, _, outputs, _ = guard
    reports = []
    node.report = reports.append
    node.goal_status_callback(status(GoalStatus.STATUS_EXECUTING))
    node.goal_status_callback(status(GoalStatus.STATUS_CANCELING))
    node.tick()
    assert outputs[-1] == 0.0
    node.goal_status_callback(status(GoalStatus.STATUS_CANCELED))
    node.tick()
    assert reports[-1] == 'ready_goal_canceled' and not node.active_goals
