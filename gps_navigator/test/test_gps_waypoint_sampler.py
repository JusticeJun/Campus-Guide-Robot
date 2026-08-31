import random

import pytest

from gps_navigator.gps_waypoint_sampler_node import analyze_samples


def samples_from_local(points, step_s=5.0):
    latitude = 35.0
    longitude = 129.0
    meters_per_latitude_degree = 111194.9
    meters_per_longitude_degree = 91085.6
    return [
        {
            'elapsed_s': index * step_s,
            'latitude': latitude + north / meters_per_latitude_degree,
            'longitude': longitude + east / meters_per_longitude_degree,
            'east_m': east,
            'north_m': north,
        }
        for index, (east, north) in enumerate(points)
    ]


def test_random_scatter_uses_full_session_robust_center():
    generator = random.Random(4)
    points = [
        (generator.gauss(0.0, 0.35), generator.gauss(0.0, 0.35))
        for _ in range(120)
    ]
    result = analyze_samples(samples_from_local(points))
    assert result['recommended_method'] == 'full_session_median'
    assert result['confidence'] == 'GOOD'
    assert not result['monotonic_drift']


def test_initial_convergence_selects_later_stable_window():
    generator = random.Random(8)
    points = []
    for index in range(40):
        center = 8.0 * (1.0 - index / 39.0)
        points.append((center + generator.gauss(0.0, 0.15),
                       generator.gauss(0.0, 0.15)))
    points.extend(
        (generator.gauss(0.0, 0.15), generator.gauss(0.0, 0.15))
        for _ in range(80)
    )
    result = analyze_samples(samples_from_local(points))
    assert result['recommended_method'] in {'middle_median', 'late_median'}
    assert abs(result['recommended_local_m'][0]) < 0.2
    assert result['confidence'] == 'GOOD'


def test_continuous_east_drift_is_reported_as_poor_confidence():
    points = [(index * 0.1, 0.0) for index in range(121)]
    result = analyze_samples(samples_from_local(points))
    assert result['east_drift_m_min'] == pytest.approx(1.2)
    assert result['monotonic_drift']
    assert result['confidence'] == 'POOR'
    assert 'drift' in result['warning']


def test_median_recommendation_resists_few_large_outliers():
    points = [(0.0, 0.0)] * 96 + [
        (30.0, 20.0), (30.0, 20.0), (-25.0, 15.0), (20.0, -30.0)
    ]
    result = analyze_samples(samples_from_local(points))
    assert result['recommended_local_m'] == pytest.approx((0.0, 0.0))
    assert abs(result['mean_coordinate'][0] - 35.0) > 1e-7
    assert result['median_coordinate'] == pytest.approx((35.0, 129.0))
