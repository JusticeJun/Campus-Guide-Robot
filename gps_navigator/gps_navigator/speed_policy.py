"""Apply the map-navigation speed policy after per-map configuration selection."""

import math


def apply_speed_policy(params, turn_speed=0.25):
    """Use native RPP curvature regulation; allow a bounded field comparison."""
    turn_speed = float(turn_speed)
    if not math.isfinite(turn_speed) or not 0.25 <= turn_speed <= 0.35:
        raise ValueError('turn_speed_m_s must be between 0.25 and 0.35 m/s')
    controller = params['controller_server']['ros__parameters']['FollowPath']
    if controller['plugin'] != (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    ):
        raise ValueError('Speed policy requires the RegulatedPurePursuitController')
    controller.update(
        {
            'desired_linear_vel': 0.35,
            'use_regulated_linear_velocity_scaling': True,
            'regulated_linear_scaling_min_radius': 3.0,
            'regulated_linear_scaling_min_speed': turn_speed,
        }
    )
    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0] = min(float(smoother['max_velocity'][0]), 0.4)
    return turn_speed
