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


def test_ten_degree_error_requests_maximum_steering():
    assert Navigator.steering_yaw_rate(10.0, 1.5, 10.0, 2.0) == -1.5


def test_steering_hysteresis_holds_straight_until_reentry_threshold():
    assert not Navigator.hysteresis_active(True, 5.0, 10.0, 5.0)
    assert not Navigator.hysteresis_active(False, 9.9, 10.0, 5.0)
    assert Navigator.hysteresis_active(False, 10.0, 10.0, 5.0)


def test_large_error_alignment_uses_separate_hysteresis():
    assert Navigator.hysteresis_active(True, 31.0, 45.0, 30.0)
    assert not Navigator.hysteresis_active(True, 30.0, 45.0, 30.0)
    assert Navigator.hysteresis_active(False, 45.0, 45.0, 30.0)
