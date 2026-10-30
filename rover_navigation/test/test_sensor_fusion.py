"""Independent measurements, scan visibility and Ackermann obstacle constraints."""
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import pytest
import yaml

from rover_navigation.fusion_health import MeasurementHealth, valid_motion
from rover_navigation.lidar_filter_node import crop_scan
from rover_navigation.lidar_navigation import configure_obstacles
from rover_navigation.path_validation import apply_obstacle_clearance


def test_camera_loss_keeps_lidar_but_predictions_cannot_keep_tracking_alive():
    state = MeasurementHealth(1.5)
    state.update('visual', 10, True)
    state.update('lidar', 10.1, True)
    state.update('visual', 10.2, False)
    assert state.active(10.3) == ['lidar']
    assert state.active(11.7) == []
    assert not state.update('lidar', 9, True)
    state.update('lidar', 11.8, False)
    assert state.active(11.8) == []
    state.update('visual', 11.9, True)
    assert state.active(12) == ['visual']


def test_stale_nonfinite_or_failed_odometry_is_not_fused():
    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0
    assert valid_motion(msg, 10.5)
    assert not valid_motion(msg, 12)
    msg.twist.twist.linear.x = float('nan')
    assert not valid_motion(msg, 10.5)
    msg.twist.twist.linear.x = 0.3
    msg.pose.covariance[0] = 9999
    assert not valid_motion(msg, 10.5)


def test_front_200_crop_preserves_ray_time_and_blind_rear_is_not_free_space():
    scan = LaserScan()
    scan.header.stamp.sec = 10
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / 180
    scan.time_increment = 0.001
    scan.range_min, scan.range_max = 0.15, 12.0
    scan.ranges, scan.intensities = [2.0] * 361, [1.0] * 361
    front = crop_scan(scan)
    assert len(front.ranges) in (199, 200, 201)
    assert front.angle_min == pytest.approx(-math.radians(100), abs=0.02)
    assert front.angle_max == pytest.approx(math.radians(100), abs=0.02)
    assert front.header.stamp.nanosec == pytest.approx(80000000, abs=1000001)
    assert len(front.intensities) == len(front.ranges)
    assert all(v == 2 for v in front.ranges)
    scan.ranges[180] = 0.05
    assert any(math.isnan(v) for v in crop_scan(scan).ranges)
    assert scan.header.stamp.nanosec == 0


def test_obstacles_change_both_costmaps_without_changing_speed_or_turn_radius():
    params = yaml.safe_load((Path(__file__).parents[1] / 'config/nav2_rtab.yaml').read_text())
    speed = params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']
    configure_obstacles(params)
    apply_obstacle_clearance(params, lidar_obstacles=True)
    for name in ('local_costmap', 'global_costmap'):
        c = params[name][name]['ros__parameters']
        assert c['plugins'].index('obstacle_layer') < c['plugins'].index('inflation_layer')
        assert c['obstacle_layer']['front_scan']['clearing']
        assert c['obstacle_layer']['front_scan']['inf_is_valid']
        assert c['obstacle_layer']['footprint_clearing_enabled'] == (name == 'global_costmap')
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['desired_linear_vel'] == speed
    assert controller['allow_reversing']
    assert controller['maximum_reverse_speed'] == .4
    assert params['velocity_smoother']['ros__parameters']['min_velocity'] == [-.4, 0., -.36]
    assert params['planner_server']['ros__parameters'][
        'GridBased']['minimum_turning_radius'] == 2.0
    planner = params['planner_server']['ros__parameters']['GridBased']
    assert not planner['downsample_costmap'] and planner['downsampling_factor'] == 1
    assert not planner['cache_obstacle_heuristic']
    assert planner['allow_unknown']
    assert planner['motion_model_for_search'] == 'REEDS_SHEPP'
    assert planner['reverse_penalty'] == 3.0 and planner['change_penalty'] == 0.0
    assert params['global_costmap']['global_costmap']['ros__parameters']['update_frequency'] == 5.0
    assert params['local_costmap']['local_costmap']['ros__parameters']['resolution'] == 0.05


def test_rotated_lidar_masks_vehicle_rear_even_when_sector_wraps():
    scan = LaserScan()
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = math.pi / 180
    scan.range_max = 12.0
    scan.ranges = [2.0] * 361
    front = crop_scan(scan, sensor_yaw=math.pi)
    assert math.isnan(front.ranges[180])  # laser forward is vehicle rear
    assert math.isfinite(front.ranges[0]) and math.isfinite(front.ranges[-1])


def test_lidar_bad_sideways_motion_is_rejected_without_disabling_reverse_fallback():
    from geometry_msgs.msg import Twist
    from rover_navigation.fusion_health import lidar_velocity_valid
    lidar, visual = Twist(), Twist()
    lidar.linear.x = .2
    lidar.linear.y = .9
    assert not lidar_velocity_valid(lidar, visual)
    assert not lidar_velocity_valid(lidar)  # Impossible rear-axle lateral motion.
    lidar.linear.y = 0.
    lidar.linear.x = -.35
    assert lidar_velocity_valid(lidar)
    assert lidar_velocity_valid(lidar, visual)  # Camera lag at normal start/reverse.
    lidar.linear.x = .9
    assert not lidar_velocity_valid(lidar, visual)
