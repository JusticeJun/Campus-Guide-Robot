import math

from gps_navigator.turn_radius_calibration_node import (
    fit_circle,
    gps_to_local_xy,
    heading_delta_deg,
    update_vehicle_constraints,
)


def test_heading_delta_wraps_across_north():
    assert heading_delta_deg(2.0, 358.0) == 4.0
    assert heading_delta_deg(358.0, 2.0) == -4.0


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


def test_vehicle_constraints_use_more_conservative_turn_radius():
    data = {}
    update_vehicle_constraints(data, 'LEFT', 2.5, -1.0, 1100, 0.15)
    update_vehicle_constraints(data, 'RIGHT', 3.0, 1.0, 1900, 0.15)
    constraints = data['vehicle_constraints']
    assert constraints['minimum_turn_radius_left_m'] == 2.5
    assert constraints['minimum_turn_radius_right_m'] == 3.0
    assert constraints['minimum_safe_turn_radius_m'] == 3.0
    assert constraints['maximum_curvature_1_per_m'] == 0.333333
