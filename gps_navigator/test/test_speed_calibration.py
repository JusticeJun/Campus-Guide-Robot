from mavros_msgs.msg import PositionTarget

from gps_navigator.speed_calibration_node import (
    DIAGNOSTIC_FIELDS,
    make_straight_speed_command,
    pid_throttle_demand,
)


def test_straight_command_uses_production_pipeline_with_zero_yaw_rate():
    command = make_straight_speed_command(0.40)

    assert command.coordinate_frame == PositionTarget.FRAME_BODY_NED
    assert command.velocity.x == 0.40
    assert command.yaw_rate == 0.0
    assert command.type_mask & PositionTarget.IGNORE_YAW
    assert not command.type_mask & PositionTarget.IGNORE_VX
    assert not command.type_mask & PositionTarget.IGNORE_YAW_RATE


def test_speed_trace_contains_required_diagnostic_fields():
    required = {
        'requested_speed_m_s',
        'actual_speed_m_s',
        'fcu_target_speed_m_s',
        'fcu_target_yaw_rate_rad_s',
        'pid_tuning_desired',
        'pid_tuning_achieved',
        'pid_tuning_ff',
        'pid_tuning_p',
        'pid_tuning_i',
        'pid_tuning_d',
        'pid_tuning_throttle_demand',
        'throttle_pwm',
        'steering_pwm',
    }

    assert required <= set(DIAGNOSTIC_FIELDS)


def test_pid_throttle_demand_is_sum_of_reported_controller_components():
    tuning = {'ff': 0.20, 'p': 0.03, 'i': 0.04, 'd': -0.01}

    assert pid_throttle_demand(tuning) == 0.26
