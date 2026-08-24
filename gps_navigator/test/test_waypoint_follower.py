import math

from gps_navigator.waypoint_follower_node import Navigator


def test_signed_heading_error_wraps_at_zero_degrees():
    assert Navigator.signed_heading_error(10.0, 350.0) == 20.0
    assert Navigator.signed_heading_error(350.0, 10.0) == -20.0


def test_sixty_degree_error_produces_visible_yaw_rate():
    error = Navigator.signed_heading_error(60.0, 0.0)
    yaw_rate = math.radians(error) * 0.5

    assert math.isclose(yaw_rate, math.pi / 6.0)
