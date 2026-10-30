"""Speed policy cannot be silently reverted by an old map configuration."""

from pathlib import Path

import pytest
import yaml

from rover_navigation.pixhawk_command_adapter_node import bounded_motion
from rover_navigation.speed_policy import apply_speed_policy


def test_old_map_speed_settings_are_overridden_without_changing_safety_settings():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/nav2_rtab.yaml').read_text())
    controller = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    controller['desired_linear_vel'] = 0.4
    controller['regulated_linear_scaling_min_speed'] = 0.4
    smoother['max_velocity'][0] = 0.6
    guard = config['localization_guard']['ros__parameters'].copy()
    apply_speed_policy(config)
    assert controller['desired_linear_vel'] == 0.35
    assert controller['regulated_linear_scaling_min_speed'] == 0.25
    assert controller['regulated_linear_scaling_min_radius'] == 3.0
    assert controller['use_regulated_linear_velocity_scaling']
    assert smoother['max_velocity'][0] == 0.4
    assert smoother['feedback'] == 'OPEN_LOOP'
    assert config['localization_guard']['ros__parameters'] == guard
    assert not controller['use_rotate_to_heading']
    assert not controller['allow_reversing']
    assert controller['min_approach_linear_velocity'] == 0.2
    assert smoother['max_accel'][0] == 0.5
    assert smoother['max_decel'][0] == -0.8
    apply_speed_policy(config, 0.35)
    assert controller['regulated_linear_scaling_min_speed'] == 0.35
    smoother['max_velocity'][0] = 0.25
    apply_speed_policy(config)
    assert smoother['max_velocity'][0] == 0.25


@pytest.mark.parametrize('invalid', [0.24, 0.36, float('nan'), float('inf')])
def test_field_comparison_rejects_out_of_range_floor(invalid):
    with pytest.raises(ValueError, match='turn_speed_m_s'):
        apply_speed_policy({}, invalid)


@pytest.mark.parametrize('yaw', [-2.0, 0.0, 2.0])
def test_final_cap_survives_old_override_and_zero_remains_zero(yaw):
    speed, rate = bounded_motion(0.8, yaw, 0.6, 1.7)
    assert speed == 0.4
    assert abs(rate) <= 0.4 / 1.7
    assert bounded_motion(0.0, yaw, 0.6, 1.7) == (0.0, 0.0)
    assert bounded_motion(0.2, yaw, 0.6, 1.7)[0] == 0.2
