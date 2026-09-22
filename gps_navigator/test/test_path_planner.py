import pytest

from gps_navigator.geo_utils import gps_to_local_xy
from gps_navigator.path_planner_node import (
    PathPlanner,
    nearest_edge_projection,
)


def waypoint(latitude, longitude, neighbors):
    return {
        'name': 'test',
        'latitude': latitude,
        'longitude': longitude,
        'neighbors': neighbors,
    }


def linear_graph():
    return {
        'W06': waypoint(35.0, 129.0000, ['W07']),
        'W07': waypoint(35.0, 129.0002, ['W06', 'W02']),
        'W02': waypoint(35.0, 129.0004, ['W07']),
    }


def planner_for(
    waypoints, latitude, longitude, max_distance=15.0, snap_distance=1.0,
    endpoint_margin=3.0,
):
    planner = object.__new__(PathPlanner)
    planner.waypoints = waypoints
    planner.current_latitude = latitude
    planner.current_longitude = longitude

    values = {
        'max_start_edge_distance_m': max_distance,
        'start_node_snap_distance_m': snap_distance,
        'start_edge_endpoint_margin_m': endpoint_margin,
    }

    class Parameter:
        def __init__(self, value):
            self.value = value

    planner.get_parameter = lambda name: Parameter(values[name])
    return planner


def test_mid_edge_start_selects_destination_direction():
    planner = planner_for(linear_graph(), 35.0, 129.0001)
    route, _, projection = planner.route_from_projected_start('W02')
    assert {projection['a'], projection['b']} == {'W06', 'W07'}
    assert route == ['W07', 'W02']


def test_start_before_w06_preserves_w06_as_first_graph_node():
    planner = planner_for(linear_graph(), 35.0, 128.99992)
    route, _, projection = planner.route_from_projected_start('W02')
    assert projection['raw_fraction_from_a'] < 0.0
    assert projection['selected_start'] == 'W06'
    assert route == ['W06', 'W07', 'W02']


def test_mid_edge_start_can_select_opposite_endpoint():
    planner = planner_for(linear_graph(), 35.0, 129.0001)
    route, _, _ = planner.route_from_projected_start('W06')
    assert route == ['W06']


def test_mid_w07_w02_start_can_select_w02_directly():
    planner = planner_for(linear_graph(), 35.0, 129.0003)
    route, _, projection = planner.route_from_projected_start('W02')
    assert {projection['a'], projection['b']} == {'W07', 'W02'}
    assert projection['selected_start'] == 'W02'
    assert route == ['W02']


def test_endpoint_margin_prevents_gps_noise_from_skipping_w06():
    planner = planner_for(linear_graph(), 35.0, 129.00002)
    route, _, projection = planner.route_from_projected_start('W02')
    assert projection['selected_start'] == 'W06'
    assert route == ['W06', 'W07', 'W02']


def test_start_at_existing_node_preserves_node_route():
    planner = planner_for(linear_graph(), 35.0, 129.0000)
    route, _, _ = planner.route_from_projected_start('W02')
    assert route == ['W06', 'W07', 'W02']


def test_far_from_graph_falls_back_to_nearest_waypoint():
    planner = planner_for(linear_graph(), 35.001, 129.0000, max_distance=5.0)
    route, _, projection = planner.route_from_projected_start('W02')
    assert projection is None
    assert route == ['W06', 'W07', 'W02']


def test_projection_uses_metric_point_to_segment_distance():
    graph = linear_graph()
    projection = nearest_edge_projection(graph, 35.00001, 129.0001)
    expected = gps_to_local_xy(35.0, 129.0001, 35.00001, 129.0001)
    assert projection['fraction_from_a'] == pytest.approx(0.5, abs=1e-3)
    assert projection['distance_to_edge_m'] == pytest.approx(abs(expected[1]))


def test_one_way_edge_only_allows_forward_endpoint():
    graph = linear_graph()
    graph['W07']['neighbors'].remove('W06')
    planner = planner_for(graph, 35.0, 129.0001)
    route, _, _ = planner.route_from_projected_start('W02')
    assert route == ['W07', 'W02']
