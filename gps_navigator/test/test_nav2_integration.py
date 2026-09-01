import math

import pytest

from gps_navigator.geo_utils import compass_heading_to_yaw, gps_to_local_xy
from gps_navigator.nav2_route_adapter_node import route_yaws
from gps_navigator.pixhawk_command_adapter_node import (
    bounded_motion,
    make_position_target,
)


def test_gps_projection_uses_east_north_axes():
    east = gps_to_local_xy(35.0, 129.0001, 35.0, 129.0)
    north = gps_to_local_xy(35.0001, 129.0, 35.0, 129.0)
    assert east[0] > 0.0
    assert east[1] == pytest.approx(0.0)
    assert north[0] == pytest.approx(0.0)
    assert north[1] > 0.0


@pytest.mark.parametrize(
    ('heading', 'yaw'),
    [(0.0, math.pi / 2.0), (90.0, 0.0), (180.0, -math.pi / 2.0)],
)
def test_compass_heading_to_ros_yaw(heading, yaw):
    assert compass_heading_to_yaw(heading) == pytest.approx(yaw)


def test_route_orientations_follow_outgoing_segments():
    assert route_yaws([(0.0, 0.0), (2.0, 0.0), (2.0, 3.0)]) == pytest.approx(
        [0.0, math.pi / 2.0, math.pi / 2.0]
    )


def test_command_adapter_enforces_forward_speed_and_turning_radius():
    speed, yaw_rate = bounded_motion(0.8, 2.0, 0.6, 1.7)
    assert speed == pytest.approx(0.6)
    assert yaw_rate == pytest.approx(0.6 / 1.7)
    assert bounded_motion(-0.2, 0.5, 0.6, 1.7) == (0.0, 0.0)


def test_mavros_command_uses_body_forward_velocity_and_yaw_rate():
    command = make_position_target(0.4, -0.2)
    assert command.coordinate_frame == command.FRAME_BODY_NED
    assert command.velocity.x == pytest.approx(0.4)
    assert command.velocity.y == pytest.approx(0.0)
    assert command.yaw_rate == pytest.approx(-0.2)
    assert not command.type_mask & command.IGNORE_VX
    assert not command.type_mask & command.IGNORE_VY
    assert not command.type_mask & command.IGNORE_YAW_RATE
