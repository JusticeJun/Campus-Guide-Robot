from gps_navigator.navigation_status_monitor_node import (
    NavigationStatusMonitor,
    command_alert,
    format_number,
    progress_target_index,
)


def test_progress_target_index_tracks_nav2_remaining_poses():
    assert progress_target_index(3, 3) == 0
    assert progress_target_index(3, 2) == 1
    assert progress_target_index(3, 1) == 2
    assert progress_target_index(3, 0) is None


def test_progress_target_index_is_bounded_for_initial_feedback():
    assert progress_target_index(3, 4) == 0
    assert progress_target_index(0, 1) is None


def test_format_number_handles_missing_and_signed_values():
    assert format_number(None) == 'n/a'
    assert format_number(float('nan')) == 'n/a'
    assert format_number(0.235, signed=True) == '+0.23'
    assert format_number(-0.2, signed=True) == '-0.20'


def test_first_projected_route_waypoint_is_reported_from_start():
    monitor = object.__new__(NavigationStatusMonitor)
    monitor.route = ['W07', 'W02']
    monitor.target_index = 0
    assert monitor.segment_text() == ('start->W07', 'W07')


def test_command_alert_detects_yaw_rate_sign_mismatch():
    assert command_alert(0.2, 0.15, -0.1) is not None
    assert command_alert(0.2, 0.15, 0.1) is None
    assert command_alert(None, 0.15, 0.1) is None


def test_compact_dashboard_contains_operator_fields(capsys):
    monitor = object.__new__(NavigationStatusMonitor)
    monitor.route = ['W06', 'W07', 'W02']
    monitor.target_index = 0
    monitor.waypoints = {'W06': {'name': '카페 파라다이스'}}
    monitor.latitude = 35.1353975
    monitor.longitude = 129.1041419
    monitor.heading_deg = 265.6
    monitor.nav_speed = 0.4
    monitor.nav_yaw_rate = 0.2
    monitor.smooth_speed = 0.4
    monitor.smooth_yaw_rate = 0.2
    monitor.fcu_command_speed = 0.4
    monitor.fcu_command_yaw_rate = 0.2
    monitor.steering_pwm = 1325
    monitor.throttle_pwm = 1334
    monitor.pid_desired = 13.4
    monitor.pid_achieved = 13.1
    monitor.navigation_state = 'RUNNING'
    monitor.events = ['[EVENT] FIRST NODE: W06']
    monitor.target_distance = lambda: 2.4

    monitor.draw_dashboard()

    output = capsys.readouterr().out
    assert '\033[H\033[J' in output
    assert 'ROUTE   W06 -> W07 -> W02' in output
    assert 'TARGET  W06 카페 파라다이스' in output
    assert 'CONTROL  NAV → SMOOTH → FCU' in output
    assert 'STATE   RUNNING' in output
