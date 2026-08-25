from gps_navigator.waypoint_follower_node import Navigator


def test_signed_heading_error_wraps_at_zero_degrees():
    assert Navigator.signed_heading_error(10.0, 350.0) == 20.0
    assert Navigator.signed_heading_error(350.0, 10.0) == -20.0


def test_sixty_degree_error_produces_visible_yaw_rate():
    error = Navigator.signed_heading_error(60.0, 0.0)
    yaw_rate = Navigator.steering_yaw_rate(error, 1.0, 30.0, 2.0)

    assert yaw_rate == -1.0


def test_geographic_left_turn_produces_positive_ros_yaw_rate():
    error = Navigator.signed_heading_error(300.0, 0.0)

    assert Navigator.steering_yaw_rate(error, 1.0, 30.0, 2.0) > 0.0


def test_steering_tapers_near_target_heading():
    assert Navigator.steering_yaw_rate(2.0, 1.0, 30.0, 2.0) == 0.0
    assert 0.0 > Navigator.steering_yaw_rate(10.0, 1.0, 30.0, 2.0) > -1.0


def test_near_waypoint_reaches_full_steering_at_smaller_error():
    threshold = Navigator.full_steering_error_for_distance(
        8.0, 10.0, 15.0, 30.0
    )

    assert threshold == 15.0
    assert Navigator.steering_yaw_rate(15.0, 1.5, threshold, 2.0) == -1.5


def test_distance_based_steering_threshold_interpolates():
    assert Navigator.full_steering_error_for_distance(
        15.0, 10.0, 15.0, 30.0
    ) == 22.5
    assert Navigator.full_steering_error_for_distance(
        20.0, 10.0, 15.0, 30.0
    ) == 30.0
