"""Map replacement, route-area, speed preservation and fail-closed regressions."""

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import yaml

from rover_navigation.localization_guard_node import valid_estimate
from rover_navigation.navigation_profile import (
    covered_graph,
    gps_to_map,
    load_profile,
    map_to_gps,
)
from rover_navigation.path_validation import (
    apply_obstacle_clearance, path_is_clear, shared_footprint,
)
from rover_navigation.route_area import route_points, travel_grid

ROOT = Path(__file__).resolve().parents[1]
PARAMS = str(ROOT / 'config/nav2_rtab.yaml')


@pytest.fixture
def profile_files(tmp_path):
    (tmp_path / 'map.yaml').write_text(
        yaml.safe_dump(
            {
                'image': 'map.pgm',
                'origin': [0.0, 0.0, 0.0],
                'resolution': 0.1,
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.2,
            }
        )
    )
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    (tmp_path / 'map.db').write_bytes(b'test artifact, not a localization database')
    graph = {
        k: {'latitude': 35.0, 'longitude': 129.0 + i * 0.0001, 'name': k, 'neighbors': neighbors}
        for i, (k, neighbors) in enumerate([('A', ['B']), ('B', ['A'])])
    }
    (tmp_path / 'graph.yaml').write_text(yaml.safe_dump({'waypoints': graph}))
    data = {
        'map': 'map.yaml',
        'database': 'map.db',
        'waypoints_file': 'graph.yaml',
        'alignment': {'datum': [35.0, 129.0], 'enu_to_map': [0.0, 0.0, 0.0], 'reviewed': True},
        'base_to_camera': [0.3001, 0.0175, 0.2032, 0.0, 0.0, 0.0],
        'odometry_provider': 'rgbd_odometry',
        'odometry_qualified': True,
        'route_half_width_m': 2.5,
        'covered_nodes': ['A', 'B'],
        'reviewed_edges': [['A', 'B'], ['B', 'A']],
        'rgb_topic': '/rgb',
        'depth_topic': '/depth',
        'camera_info_topic': '/info',
    }
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump(data))
    data['validated_fingerprint'] = load_profile(str(profile), 'validation', params_file=PARAMS)[
        '_fingerprint'
    ]
    profile.write_text(yaml.safe_dump(data))
    return profile, data


def test_alignment_round_trip_rotation_and_translation():
    alignment = {'datum': [35.0, 129.0], 'enu_to_map': [12.0, -8.0, math.pi / 2]}
    x, y = gps_to_map(35.0, 129.0001, alignment)
    assert x == pytest.approx(12.0)
    assert y > -8.0
    assert map_to_gps(x, y, alignment) == pytest.approx([35.0, 129.0001])


@pytest.mark.parametrize('artifact', ['map.yaml', 'map.pgm', 'map.db', 'graph.yaml'])
def test_replacing_any_artifact_invalidates_drive(profile_files, artifact):
    profile, _ = profile_files
    assert load_profile(str(profile), 'drive', params_file=PARAMS)
    path = profile.parent / artifact
    path.write_bytes(path.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='changed or not commissioned'):
        load_profile(str(profile), 'drive', params_file=PARAMS)


def test_calibration_speed_config_and_odom_qualification_are_bound(profile_files, tmp_path):
    profile, data = profile_files
    params = tmp_path / 'params.yaml'
    params.write_text(
        Path(PARAMS).read_text().replace('desired_linear_vel: 0.35', 'desired_linear_vel: 0.5')
    )
    with pytest.raises(ValueError, match='changed or not commissioned'):
        load_profile(str(profile), 'drive', params_file=str(params))
    data['odometry_qualified'] = False
    profile.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='odometry'):
        load_profile(str(profile), 'drive', params_file=PARAMS)
    assert load_profile(str(profile), 'validation', params_file=PARAMS)
    data['alignment']['enu_to_map'][0] = float('nan')
    profile.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='finite'):
        load_profile(str(profile), 'validation', params_file=PARAMS)


def test_symlink_selection_and_map_override(profile_files, tmp_path):
    profile, _ = profile_files
    selected = tmp_path / 'selection' / 'current.yaml'
    selected.parent.mkdir()
    selected.symlink_to(profile)
    assert load_profile(str(selected), 'drive', params_file=PARAMS)
    with pytest.raises(ValueError, match='planner-only'):
        load_profile(str(profile), 'drive', str(tmp_path / 'map.yaml'))


def test_map_frame_change_reprojects_same_graph_without_manual_points(profile_files):
    profile, data = profile_files
    graph = yaml.safe_load((profile.parent / 'graph.yaml').read_text())['waypoints']
    points = route_points(['A', 'B'], covered_graph(graph, data), data['alignment'])
    data['alignment']['enu_to_map'] = [10.0, 20.0, math.pi / 2]
    rotated = route_points(['A', 'B'], covered_graph(graph, data), data['alignment'])
    for (x, y), (rx, ry) in zip(points, rotated):
        assert (rx, ry) == pytest.approx((10 - y, 20 + x))
    assert graph['A']['neighbors'] == ['B']


def grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 100
    msg.info.resolution = 0.1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 10000
    return msg


def pose(x, y):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation.w = 1.0
    return msg


def test_travel_area_blocks_shortcut_but_allows_lateral_path_choice():
    msg = grid()
    msg.data[22 * 100 + 22] = -1
    msg.data = travel_grid(msg.info, msg.data, [(2, 2), (2, 7), (7, 7)], 1.0)
    cells = np.array(msg.data).reshape(100, 100)
    assert cells[22, 22] == -1  # Never turn original unknown into free.
    assert cells[50, 25] == 0  # Offset from centerline is allowed.
    assert cells[40, 40] == 100  # Diagonal shortcut is outside selected route.


def test_footprint_and_interpolation_catch_center_free_collisions():
    msg = grid()
    foot = [[0.41, 0.135], [0.41, -0.135], [-0.07, -0.135], [-0.07, 0.135]]
    assert path_is_clear([pose(2, 2), pose(4, 2)], msg, foot, 0.05)
    msg.data[21 * 100 + 30] = 100  # Along swept footprint, not endpoints/centerline.
    assert not path_is_clear([pose(2, 2), pose(4, 2)], msg, foot, 0.05)
    msg.data[21 * 100 + 30] = -1
    assert not path_is_clear([pose(2, 2), pose(4, 2)], msg, foot, 0.05)
    assert not path_is_clear([pose(0, 0)], msg, foot, 0.05)


def test_localization_quality_rejects_unregistered_invalid_or_uncertain_pose():
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.orientation.w = 1.0
    assert not valid_estimate(msg, 'map', 0.25, 0.12)
    for index in (0, 7, 35):
        msg.pose.covariance[index] = 0.01
    assert valid_estimate(msg, 'map', 0.25, 0.12)
    msg.pose.covariance[0] = 9999.0
    assert not valid_estimate(msg, 'map', 0.25, 0.12)
    msg.pose.covariance[0] = 0.01
    msg.pose.pose.position.x = float('nan')
    assert not valid_estimate(msg, 'map', 0.25, 0.12)


def test_static_costmaps_and_existing_vehicle_settings_are_preserved():
    config = yaml.safe_load((ROOT / 'config/nav2_rtab.yaml').read_text())
    legacy = yaml.safe_load((ROOT / 'config/nav2_gps.yaml').read_text())
    controller = config['controller_server']['ros__parameters']['FollowPath'].copy()
    previous = legacy['controller_server']['ros__parameters']['FollowPath'].copy()
    assert controller.pop('desired_linear_vel') == 0.35
    assert controller.pop('regulated_linear_scaling_min_speed') == 0.25
    previous.pop('desired_linear_vel')
    previous.pop('regulated_linear_scaling_min_speed')
    assert controller == previous
    a = config['velocity_smoother']['ros__parameters'].copy()
    b = legacy['velocity_smoother']['ros__parameters'].copy()
    a.pop('odom_topic')
    b.pop('odom_topic')
    assert a == b
    for name in ('local_costmap', 'global_costmap'):
        p = config[name][name]['ros__parameters']
        assert p['plugins'] == ['static_layer', 'route_layer', 'inflation_layer']
        assert p['track_unknown_space']
    planner = config['planner_server']['ros__parameters']['GridBased']
    assert not planner['allow_unknown']
    assert planner['minimum_turning_radius'] == 2.0
    assert planner['motion_model_for_search'] == 'DUBIN'
    assert not planner['smooth_path']


def test_guard_latches_on_loss_and_never_auto_resumes():
    from types import SimpleNamespace

    from geometry_msgs.msg import Twist
    from rclpy.time import Time

    from rover_navigation.localization_guard_node import LocalizationGuard

    guard = object.__new__(LocalizationGuard)
    now, reason, outputs = [10.0], [''], []
    guard.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    guard.get_parameter = lambda name: SimpleNamespace(
        value={'stable_duration_s': 3.0, 'path_timeout_s': 4.0,
               'maximum_reverse_speed_m_s': 0.0}[name]
    )
    guard.health_reason = lambda: reason[0]
    guard.report = lambda text: None
    guard.ready_pub = SimpleNamespace(publish=lambda msg: None)
    guard.stop_latched_pub = SimpleNamespace(publish=lambda msg: None)
    guard.publisher = SimpleNamespace(publish=lambda msg: outputs.append(msg.linear.x))
    guard.ever_ready = guard.latched = guard.recovering = False
    guard.odom_rejection = ''
    guard.resume_after = None
    guard.recovery_started = None
    guard.recovery_cause = guard.latch_reason = guard.goal_outcome = ''
    guard.goal_source = 'route'
    guard.stable_since = Time(seconds=6.0)
    guard.route_time = guard.command_time = guard.path_time = Time(seconds=10.0)
    guard.path_clear = True
    guard.command = Twist()
    guard.command.linear.x = 0.4
    guard.tick()
    assert outputs[-1] == 0.4
    reason[0], now[0] = 'stale_tf_map_odom', 10.1
    guard.tick()
    assert outputs[-1] == 0.0
    assert guard.latched
    reason[0] = ''
    for value in (10.2, 14.0):
        now[0] = value
        guard.route_time = guard.command_time = guard.path_time = Time(seconds=value)
        guard.tick()
    assert outputs[-1] == 0.0


def test_direct_destination_does_not_require_nearby_graph_edge(profile_files):
    import json
    from types import SimpleNamespace

    from rclpy.clock import ClockType
    from rclpy.time import Time

    from rover_navigation.path_planner_node import PathPlanner

    filename, profile = profile_files
    graph = yaml.safe_load((filename.parent / 'graph.yaml').read_text())['waypoints']
    planner = object.__new__(PathPlanner)
    planner.profile, planner.waypoints = profile, covered_graph(graph, profile)
    planner.localization_ready = True
    planner.readiness_time = Time(seconds=10.0, clock_type=ClockType.ROS_TIME)
    planner.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(seconds=10.0, clock_type=ClockType.ROS_TIME)
    )
    planner.get_logger = lambda: SimpleNamespace(info=lambda m: None, error=lambda m: None)
    planner.get_parameter = lambda name: SimpleNamespace(
        value={
            'route_policy': 'direct',
            'destination': 'B',
            'max_start_edge_distance_m': 0.1,
        }[name]
    )
    tf = SimpleNamespace(
        header=SimpleNamespace(stamp=Time(seconds=10.0, clock_type=ClockType.ROS_TIME).to_msg()),
        transform=SimpleNamespace(translation=SimpleNamespace(x=4.0, y=30.0)),
    )
    planner.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: tf)
    planner.last_destination = None
    messages = []
    planner.route_pub = SimpleNamespace(publish=lambda m: messages.append(m))
    planner.plan_if_needed()
    route = json.loads(messages[0].data)
    assert route['route'] == ['B']
    assert route['policy'] == 'direct'
    assert route['entry_edge'] is None


def test_sensor_validation_can_precede_geographic_alignment(profile_files):
    profile, data = profile_files
    data['alignment'] = None
    data['odometry_qualified'] = False
    profile.write_text(yaml.safe_dump(data))
    assert load_profile(str(profile), 'validation', params_file=PARAMS, route_required=False)
    with pytest.raises(ValueError, match='alignment.datum'):
        load_profile(str(profile), 'drive', params_file=PARAMS)


def test_corrected_vehicle_geometry_and_camera_reference():
    expected = [[0.41, 0.135], [0.41, -0.135], [-0.07, -0.135], [-0.07, 0.135]]
    for filename in ('nav2_rtab.yaml', 'nav2_gps.yaml'):
        config = yaml.safe_load((ROOT / 'config' / filename).read_text())
        footprint, _ = shared_footprint(config)
        assert footprint == expected
    profile = yaml.safe_load((ROOT / 'config/navigation_profile.example.yaml').read_text())
    assert profile['base_to_camera'] == pytest.approx(
        [0.3048 - 0.0047, 0.0175, 0.2032, 0.0, 0.0, 0.0]
    )


def test_rear_axle_footprint_detects_front_and_rotated_side_obstacle():
    footprint, padding = shared_footprint(yaml.safe_load(Path(PARAMS).read_text()))
    msg = grid()
    assert path_is_clear([pose(2, 2)], msg, footprint, padding)
    msg.data[20 * 100 + 24] = 100  # 0.45 m ahead: front plus padding.
    assert not path_is_clear([pose(2, 2)], msg, footprint, padding)
    msg.data[20 * 100 + 24] = 0
    p = pose(2, 2)
    p.pose.orientation.z = p.pose.orientation.w = math.sqrt(0.5)
    msg.data[24 * 100 + 20] = 100  # Same front extent after a 90-degree turn.
    assert not path_is_clear([p], msg, footprint, padding)


def test_mismatched_costmap_geometry_is_rejected():
    config = yaml.safe_load(Path(PARAMS).read_text())
    config['local_costmap']['local_costmap']['ros__parameters'][
        'footprint'
    ] = '[[0.3,0.2],[0.3,-0.2],[-0.1,-0.2],[-0.1,0.2]]'
    with pytest.raises(ValueError, match='identical footprint'):
        shared_footprint(config)


def test_corrected_width_excludes_distant_side_cell_but_checks_near_side():
    footprint, _ = shared_footprint(yaml.safe_load(Path(PARAMS).read_text()))
    padding = 0.05  # Test physical geometry separately from the navigation clearance.
    msg = grid()
    msg.data[23 * 100 + 20] = 100  # 0.35 m left: outside corrected body and margins.
    assert path_is_clear([pose(2, 2)], msg, footprint, padding)
    msg.data[21 * 100 + 20] = 100  # 0.15 m left: intersects body/padding.
    assert not path_is_clear([pose(2, 2)], msg, footprint, padding)


@pytest.mark.parametrize('width,allowed', [(0.6, False), (0.8, True)])
@pytest.mark.parametrize('old_padding', [0.05, 0.55])
def test_clearance_rejects_tight_corridor_but_accepts_wide_route(width, allowed, old_padding):
    config = yaml.safe_load(Path(PARAMS).read_text())
    # Simulate a map snapshot saved before the clearance policy.
    for name in ('local_costmap', 'global_costmap'):
        config[name][name]['ros__parameters']['footprint_padding'] = old_padding
    apply_obstacle_clearance(config)
    footprint, padding = shared_footprint(config)
    msg = grid()
    msg.info.resolution = 0.05
    # Free corridor centered on y=2; occupied space beyond both sides.
    cells = np.zeros((100, 100), dtype=int)
    yy = (np.arange(100) + 0.5) * 0.05
    cells[abs(yy - 2) >= width / 2, :] = 100
    msg.data = cells.flatten().tolist()
    assert path_is_clear([pose(2, 2), pose(3, 2)], msg, footprint, padding) is allowed


def test_guard_rejects_camera_parent_conflict():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    from rover_navigation.localization_guard_node import LocalizationGuard

    guard = object.__new__(LocalizationGuard)
    guard.tf_parent_conflict = False
    guard.invalid_tf = False
    guard.invalid_tf_frames = set()
    guard.tick = lambda: None
    tf = TransformStamped()
    tf.transform.rotation.w = 1.0
    tf.child_frame_id = 'camera_link'
    tf.header.frame_id = 'base_link'
    guard.tf_callback(TFMessage(transforms=[tf]))
    assert not guard.tf_parent_conflict
    tf.header.frame_id = 'odom'
    guard.tf_callback(TFMessage(transforms=[tf]))
    assert guard.tf_parent_conflict
    tf.header.frame_id = 'base_link'
    guard.tf_callback(TFMessage(transforms=[tf]))
    assert guard.tf_parent_conflict  # A later correct edge does not clear conflict.


def test_guard_requires_camera_tf_even_with_valid_localization():
    from types import SimpleNamespace

    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from rtabmap_msgs.msg import Info
    from tf2_ros import TransformException

    from rover_navigation.localization_guard_node import LocalizationGuard

    guard = object.__new__(LocalizationGuard)
    guard.loc = PoseWithCovarianceStamped()
    guard.odom = Odometry()
    for msg, frame in ((guard.loc, 'map'), (guard.odom, 'odom')):
        msg.header.frame_id = frame
        msg.pose.pose.orientation.w = 1.0
        for index in (0, 7, 35):
            msg.pose.covariance[index] = 0.01
    guard.odom.child_frame_id = 'base_link'
    guard.odom_rejection = ''
    guard.odom_time = None  # Receipt freshness is stubbed below in this TF-only test.
    guard.get_parameter = lambda name: SimpleNamespace(value=0.25)
    guard.fresh = lambda *args, **kwargs: True
    guard.recent = lambda *args: True
    guard.rtab_info = Info()
    guard.rtab_info.header.frame_id = 'map'
    guard.rtab_info_time = None
    guard.tf_parent_conflict = False
    guard.invalid_tf = False
    guard.lifecycle_clients = {}
    missing_camera = [True]

    def lookup(parent, child, stamp):
        if child == 'camera_link' and missing_camera[0]:
            raise TransformException('missing camera extrinsic')
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        return tf

    guard.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    assert guard.health_reason() == 'missing_tf'
    missing_camera[0] = False
    assert guard.health_reason() == ''
