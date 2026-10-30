"""Stored obstacles keep their clearance when live obstacle clearance shrinks."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from rover_navigation.path_validation import (
    apply_obstacle_clearance, path_is_clear, shared_footprint,
)
from rover_navigation.static_clearance_node import buffered_cells


def test_live_and_stored_corridor_share_margin_without_double_buffer():
    params = yaml.safe_load((Path(__file__).parents[1] / 'config/nav2_rtab.yaml').read_text())
    apply_obstacle_clearance(params, lidar_obstacles=True)
    footprint, padding = shared_footprint(params)
    assert padding == 0.20
    for name in ('local_costmap', 'global_costmap'):
        assert params[name][name]['ros__parameters']['static_layer']['map_topic'] == (
            '/navigation/static_clearance_map')
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 100
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    cells = np.zeros((100, 100), np.int8)
    rows = (np.arange(100) + 0.5) * 0.05
    cells[abs(rows - 2.0) >= 0.45, :] = 100  # 0.9 m corridor
    grid.data = cells.flatten().tolist()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = pose.pose.position.y = 2.0
    pose.pose.orientation.w = 1.0
    assert path_is_clear([pose], grid, footprint, padding)
    buffered = buffered_cells(grid.data, 100, 100, 0.05, 0.0)
    grid.data = buffered.flatten().tolist()
    assert path_is_clear([pose], grid, footprint, padding)


def test_static_buffer_preserves_unknown_and_does_not_change_input():
    cells = np.zeros((60, 60), np.int8)
    cells[30, 30], cells[30, 29] = 100, -1
    before = cells.copy()
    result = buffered_cells(cells.flatten(), 60, 60, 0.05, 0.15)
    assert np.array_equal(cells, before)
    assert result[30, 29] == -1
    assert result[30, 33] == 100
    assert result[30, 40] == 0
    # Rotating the map/rover cannot remove the extra Euclidean margin.
    for angle in np.linspace(0, 2 * math.pi, 50):
        dx, dy = round(3 * math.cos(angle)), round(3 * math.sin(angle))
        assert result[30 + dy, 30 + dx] == 100


def test_invalid_static_map_is_not_published_as_free_space():
    with pytest.raises(ValueError):
        buffered_cells([0], 2, 2, 0.05, 0.15)
