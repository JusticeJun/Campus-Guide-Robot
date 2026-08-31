import math
import struct

from mavros_msgs.msg import Mavlink

from gps_navigator.turn_radius_calibration_node import (
    DIAGNOSTIC_FIELDS,
    compass_delta_to_ros_yaw_rate,
    decode_pid_tuning,
    fit_circle,
    gps_to_local_xy,
    heading_delta_deg,
    make_body_ned_velocity_yaw_rate_command,
    update_vehicle_constraints,
    yaw_rate_for_direction,
)
from mavros_msgs.msg import PositionTarget


def test_heading_delta_wraps_across_north():
    assert heading_delta_deg(2.0, 358.0) == 4.0
    assert heading_delta_deg(358.0, 2.0) == -4.0


def test_compass_rate_is_converted_to_ros_yaw_sign_and_wraps():
    assert math.isclose(
        compass_delta_to_ros_yaw_rate(2.0, 358.0, 1.0),
        -math.radians(4.0),
    )
    assert math.isclose(
        compass_delta_to_ros_yaw_rate(358.0, 2.0, 1.0),
        math.radians(4.0),
    )
    assert compass_delta_to_ros_yaw_rate(2.0, 358.0, 0.0) is None


def test_diagnostic_trace_contains_required_control_path_fields():
    required = {
        'monotonic_time_ns',
        'ros_time_ns',
        'source',
        'event',
        'publish_sequence',
        'requested_velocity_x_m_s',
        'requested_yaw_rate_rad_s',
        'actual_forward_speed_m_s',
        'fcu_target_velocity_x_m_s',
        'fcu_target_yaw_rate_rad_s',
        'imu_yaw_rate_rad_s',
        'compass_heading_deg',
        'heading_derived_yaw_rate_rad_s',
        'steering_servo_pwm',
        'throttle_servo_pwm',
        'pid_tuning_axis',
        'pid_tuning_desired',
        'pid_tuning_achieved',
        'pid_tuning_ff',
        'pid_tuning_p',
        'pid_tuning_i',
        'pid_tuning_d',
        'vehicle_mode',
        'armed',
        'setpoint_publisher_count',
    }
    assert required <= set(DIAGNOSTIC_FIELDS)


def test_pid_tuning_mavlink_payload_is_decoded():
    payload = struct.pack('<6fB', 0.6, 0.55, 0.2, 0.1, 0.05, 0.01, 2)
    payload += b'\0' * (32 - len(payload))
    msg = Mavlink()
    msg.framing_status = Mavlink.FRAMING_OK
    msg.msgid = 194
    msg.len = 25
    msg.payload64 = list(struct.unpack('<4Q', payload))

    result = decode_pid_tuning(msg)

    assert result['axis'] == 2
    assert math.isclose(result['desired'], 0.6, rel_tol=1e-6)
    assert math.isclose(result['achieved'], 0.55, rel_tol=1e-6)
    assert math.isclose(result['ff'], 0.2, rel_tol=1e-6)
    assert math.isclose(result['p'], 0.1, rel_tol=1e-6)
    assert math.isclose(result['i'], 0.05, rel_tol=1e-6)
    assert math.isclose(result['d'], 0.01, rel_tol=1e-6)


def test_yaw_rate_direction_matches_body_ned_convention():
    assert yaw_rate_for_direction('LEFT', 2.5) == 2.5
    assert yaw_rate_for_direction('RIGHT', 2.5) == -2.5


def test_calibration_command_uses_production_body_ned_pipeline():
    command = make_body_ned_velocity_yaw_rate_command(0.20, 2.5)
    assert command.coordinate_frame == PositionTarget.FRAME_BODY_NED
    assert command.velocity.x == 0.20
    assert command.yaw_rate == 2.5
    assert command.type_mask & PositionTarget.IGNORE_YAW
    assert not command.type_mask & PositionTarget.IGNORE_VX
    assert not command.type_mask & PositionTarget.IGNORE_YAW_RATE


def test_gps_to_local_xy_uses_east_and_north_axes():
    east = gps_to_local_xy(35.0, 129.0001, 35.0, 129.0)
    north = gps_to_local_xy(35.0001, 129.0, 35.0, 129.0)
    assert east[0] > 9.0
    assert abs(east[1]) < 1e-9
    assert north[1] > 11.0
    assert abs(north[0]) < 1e-9


def test_circle_fit_recovers_known_radius():
    center_x = 3.0
    center_y = -2.0
    radius = 4.5
    points = [
        (
            center_x + radius * math.cos(math.radians(angle)),
            center_y + radius * math.sin(math.radians(angle)),
        )
        for angle in range(0, 360, 10)
    ]
    fitted_x, fitted_y, fitted_radius = fit_circle(points)
    assert math.isclose(fitted_x, center_x, abs_tol=1e-9)
    assert math.isclose(fitted_y, center_y, abs_tol=1e-9)
    assert math.isclose(fitted_radius, radius, abs_tol=1e-9)


def test_vehicle_constraints_separate_physical_and_unset_planning_limits():
    data = {}
    update_vehicle_constraints(data, 'LEFT', 2.5, -1.0, 1100, 0.15)
    update_vehicle_constraints(data, 'RIGHT', 3.0, 1.0, 1900, 0.15)
    constraints = data['vehicle_constraints']
    assert constraints['minimum_turn_radius_left_m'] == 2.5
    assert constraints['minimum_turn_radius_right_m'] == 3.0
    assert constraints['physical_min_turn_radius_m'] == 3.0
    assert constraints['planning_radius_margin_ratio'] is None
    assert constraints['planning_min_turn_radius_m'] is None
    assert constraints['maximum_physical_curvature_1_per_m'] == 0.333333
    assert constraints['maximum_planning_curvature_1_per_m'] is None


def test_vehicle_constraints_apply_configured_planning_margin():
    data = {
        'vehicle_constraints': {
            'minimum_safe_turn_radius_m': 3.0,
            'maximum_curvature_1_per_m': 0.333333,
        }
    }
    update_vehicle_constraints(
        data,
        'LEFT',
        2.5,
        1.5,
        1100,
        0.20,
        planning_radius_margin_ratio=0.10,
    )
    update_vehicle_constraints(
        data,
        'RIGHT',
        3.0,
        -1.5,
        1900,
        0.20,
        planning_radius_margin_ratio=0.10,
    )
    constraints = data['vehicle_constraints']
    assert constraints['planning_radius_margin_ratio'] == 0.10
    assert constraints['physical_min_turn_radius_m'] == 3.0
    assert constraints['planning_min_turn_radius_m'] == 3.3
    assert constraints['maximum_physical_curvature_1_per_m'] == 0.333333
    assert constraints['maximum_planning_curvature_1_per_m'] == 0.30303
    assert 'minimum_safe_turn_radius_m' not in constraints
    assert 'maximum_curvature_1_per_m' not in constraints
    assert data['calibration']['left'] == {
        'command_pipeline': 'guided_body_ned_velocity_yaw_rate',
        'requested_yaw_rate_rad_s': 1.5,
        'actual_steering_pwm': 1100,
        'actual_steering_pwm_min': None,
        'actual_steering_pwm_max': None,
        'speed_command_m_s': 0.20,
    }
