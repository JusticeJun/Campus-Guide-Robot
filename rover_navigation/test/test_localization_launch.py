"""Expand real launch files without processes; exercise DB exit and RGB-D wiring."""

import os
import json
import runpy
from pathlib import Path
import shutil
import shlex
from unittest.mock import patch

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, IncludeLaunchDescription
from launch.actions.execute_local import ExecuteLocal
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_integrated_navigation_starts_each_sensor_and_tf_once(tmp_path):
    result, commands = expand(tmp_path, mode='drive', local_odometry='fused',
                              start_lidar='true', lidar_port='/dev/test_lidar', lidar_yaw='0.1')
    assert result == 0
    for name in ('mavros_node', 'realsense2_camera_node', 'rplidar_node',
                 'imu_filter_madgwick_node', 'front_scan'):
        process(commands, name)  # Exactly one executable.
    transforms = [cmd for cmd, _ in commands if Path(cmd[0]).name == 'static_transform_publisher']
    assert len(transforms) == 2
    children = {cmd[cmd.index('--child-frame-id') + 1]: cmd for cmd in transforms}
    assert set(children) == {'camera_link', 'laser'}
    _, lidar = process(commands, 'rplidar_node')
    assert lidar['serial_port'] == '/dev/test_lidar'
    _, scan = process(commands, 'front_scan')
    assert scan['sensor_yaw'] == 0.1
    cmd = children['laser']
    assert float(cmd[cmd.index('--yaw') + 1]) == 0.1


def test_fused_odometry_and_obstacles_use_validated_health_and_filtered_commands(tmp_path):
    result, commands = expand(tmp_path, mode='drive', local_odometry='fused')
    assert result == 0
    vo_cmd, vo = process(commands, 'rgbd_odometry')
    assert not vo['publish_tf']
    assert 'odom:=/vo/odom' in vo_cmd
    _, lidar = process(commands, 'icp_odometry')
    assert not lidar['publish_tf']
    assert lidar['Icp/PointToPlaneLowComplexityStrategy'] == '0'
    _, ekf = process(commands, 'ekf_node')
    assert not ekf['publish_tf']
    process(commands, 'fusion_inputs')
    for node in ('localization_guard', 'external_navigation'):
        cmd, _ = process(commands, node)
        assert '/rtabmap/odom_info_lite:=/localization/odom_info' in cmd
    trace_cmd, _ = process(commands, 'drive_trace')
    assert '/rtabmap/odom_info_lite:=/vo/odom_info_lite' in trace_cmd
    _, adapter = process(commands, 'pixhawk_command_adapter')
    assert adapter['cmd_vel_topic'] == '/cmd_vel_vehicle'
    _, smoother = process(commands, 'velocity_smoother')
    assert smoother['min_velocity'][0] == -.4
    assert adapter['require_scan'] is True
    process(commands, 'collision_monitor')
    _, navigator = process(commands, 'bt_navigator')
    assert navigator['default_nav_to_pose_bt_xml'].endswith('/navigate_lidar.xml')
    _, adapter = process(commands, 'pixhawk_command_adapter')
    _, guard = process(commands, 'localization_guard')
    assert adapter['maximum_reverse_speed_m_s'] == .4
    assert guard['maximum_reverse_speed_m_s'] == .4
    _, planner = process(commands, 'planner_server')
    assert not planner['GridBased']['cache_obstacle_heuristic']
    _, buffer = process(commands, 'static_clearance')
    assert buffer['extra_margin_m'] == 0.0
    _, guard = process(commands, 'localization_guard')
    assert guard['footprint_padding'] == 0.20


def test_camera_map_keeps_camera_feedback_with_independent_lidar_avoidance(tmp_path):
    result, commands = expand(tmp_path, mode='drive', local_odometry='auto',
                              lidar_obstacles='true', start_lidar='true')
    assert result == 0
    _, vo = process(commands, 'rgbd_odometry')
    assert vo['publish_tf']
    names = {Path(cmd[0]).name for cmd, _ in commands}
    assert not {'icp_odometry', 'ekf_node', 'fusion_inputs'} & names
    process(commands, 'rplidar_node')
    process(commands, 'collision_monitor')
    for node in ('localization_guard', 'external_navigation'):
        cmd, _ = process(commands, node)
        assert '/rtabmap/odom_info_lite:=/localization/odom_info' not in cmd
    _, adapter = process(commands, 'pixhawk_command_adapter')
    assert adapter['require_scan']
    assert adapter['cmd_vel_topic'] == '/cmd_vel_vehicle'
    _, planner = process(commands, 'planner_server')
    assert planner['GridBased']['motion_model_for_search'] == 'REEDS_SHEPP'
    _, navigator = process(commands, 'bt_navigator')
    assert navigator['default_nav_to_pose_bt_xml'].endswith('/navigate_lidar.xml')


def expand(tmp_path, *, copy_result=0, memory_seeded=True, memory_result=0,
           external_sensors=False, **overrides):
    source = tmp_path / 'map'
    source.mkdir(exist_ok=True)
    if not (source / 'rtabmap.db').exists():
        (source / 'rtabmap.db').write_bytes(b'launch wiring fixture')
    (source / 'rtabmap.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    (source / 'rtabmap.yaml').write_text(
        yaml.safe_dump({'image': 'rtabmap.pgm', 'origin': [0.0, 0.0, 0.0], 'resolution': 0.05})
    )
    commands = []

    def capture(action, context):
        assert context.environment['RMW_IMPLEMENTATION'] == os.environ.get(
            'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'
        )
        assert context.environment.get('CYCLONEDDS_URI')
        cmd = [perform_substitutions(context, part) for part in action.cmd]
        params = {}
        for i, part in enumerate(cmd):
            if part == '--params-file':
                # Humble's launch_ros serializes some sequence parameters as
                # Python tuples; FullLoader accepts that generated YAML.
                for entry in yaml.full_load(Path(cmd[i + 1]).read_text()).values():
                    params.update(entry.get('ros__parameters', {}))
        for i, part in enumerate(cmd):
            if part == '-p':
                key, value = cmd[i + 1].split(':=', 1)
                params[key] = yaml.safe_load(value)
        commands.append((cmd, params))
        if cmd[:3] == ['python3', '-m', 'rover_navigation.localization_memory']:
            if memory_result == 0:
                Path(cmd[cmd.index('--report') + 1]).write_text(
                    json.dumps({'seeded': memory_seeded}))
            return [EmitEvent(event=ProcessExited(
                action=action, name='memory_prepare', cmd=cmd, cwd=None, env=None,
                pid=12346, returncode=memory_result))]
        if cmd[0] == 'cp':
            if copy_result == 0:
                shutil.copyfile(cmd[-2], cmd[-1])
            return [
                EmitEvent(
                    event=ProcessExited(
                        action=action,
                        name='database_copy',
                        cmd=cmd,
                        cwd=None,
                        env=None,
                        pid=12345,
                        returncode=copy_result,
                    )
                )
            ]
        return []

    settings = {
        'mode': 'validation',
        'map_dir': str(source),
        'log_root': str(tmp_path / 'sessions'),
        'database_cache_root': str(tmp_path / 'cache'),
        **({} if external_sensors else {'start_camera': 'true', 'start_mavros': 'true'}),
        'rviz': 'true',
        'localization_memory': 'auto',
        **overrides,
    }
    service = LaunchService()
    service.include_launch_description(
        LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(ROOT / 'launch/navigation.launch.py')),
                    launch_arguments=settings.items(),
                )
            ]
        )
    )
    try:
        with patch.object(ExecuteLocal, 'execute', capture):
            result = service.run()
    finally:
        lease = service.context.get_locals_as_dict().get('navigation_database_lease')
        if lease:
            lease.close()
    return result, commands


def process(commands, executable):
    found = [(cmd, params) for cmd, params in commands if Path(cmd[0]).name == executable]
    assert len(found) == 1
    return found[0]


@pytest.mark.parametrize('mode', ['validation', 'drive'])
def test_processed_rgbd_and_standard_pose_paths(tmp_path, mode):
    result, commands = expand(tmp_path, mode=mode)
    assert result == 0
    _, odom = process(commands, 'rgbd_odometry')
    rtab_command, rtab = process(commands, 'rtabmap')
    assert odom['always_process_most_recent_frame'] is True
    assert odom['wait_imu_to_init'] is True
    assert odom['keep_color'] is True
    assert odom['approx_sync_max_interval'] == 0.02
    assert odom['odom_frame_id'] == 'odom' and odom['publish_tf'] is True
    # Keep the large-image path Reliable without requesting an incompatible
    # Reliable subscription from the Best Effort MAVROS IMU publisher.
    assert odom['qos'] == 1 and odom['qos_camera_info'] == 1
    assert odom['qos_imu'] == 2
    assert rtab['qos_image'] == 1 and rtab['qos_odom'] == 1
    assert rtab['qos_imu'] == 2
    assert rtab['subscribe_rgbd'] is True
    assert rtab['subscribe_odom_info'] is False
    assert rtab['Mem/IncrementalMemory'] == 'false'
    assert rtab['Mem/InitWMWithAllNodes'] == 'true'
    assert rtab['map_cache_loaded_on_init'] is False
    assert 'rgbd_image:=/rtabmap/odom_rgbd_image_relay' in rtab_command
    relay_command, relay = process(commands, 'rgbd_relay')
    assert relay['qos'] == 1
    assert 'rgbd_image:=/rtabmap/odom_rgbd_image' in relay_command
    assert 'rgbd_image_relay:=/rtabmap/odom_rgbd_image_relay' in relay_command
    assert 'initialpose:=/initialpose' in rtab_command
    _, camera = process(commands, 'realsense2_camera_node')
    assert camera['enable_sync'] is True
    assert camera['rgb_camera.color_profile'] == '1280x720x15'
    assert camera['depth_module.depth_profile'] == '848x480x15'
    assert camera['enable_gyro'] and camera['enable_accel']
    assert camera['gyro_fps'] == 200 and camera['accel_fps'] == 100
    filter_command, imu_filter = process(commands, 'imu_filter_madgwick_node')
    assert imu_filter['publish_tf'] is False and imu_filter['use_mag'] is False
    assert 'imu/data_raw:=/camera/camera/imu' in filter_command
    assert 'imu:=/camera/imu' in rtab_command
    process(commands, 'imu_telemetry')
    # VO-specific CLI overrides must not leak into map appearance extraction.
    odom_command, _ = process(commands, 'rgbd_odometry')

    def last_argument(cmd, name):
        cmd = [token for part in cmd for token in shlex.split(part)]
        return cmd[max(i for i, arg in enumerate(cmd) if arg == name) + 1]

    assert last_argument(odom_command, '--Odom/ImageDecimation') == '2'
    assert last_argument(odom_command, '--Vis/MaxDepth') == '8.0'
    assert last_argument(rtab_command, '--Vis/MaxDepth') == '15.0'
    assert last_argument(rtab_command, '--Kp/MaxDepth') == '0'
    assert last_argument(rtab_command, '--Mem/UseOdomFeatures') == 'false'
    process(commands, 'static_transform_publisher')
    _, guard = process(commands, 'localization_guard')
    assert guard['enable_coarse_hint_handoff'] is False
    assert guard['native_localization_only'] is True
    assert guard['stable_duration_s'] == 0.5
    assert guard['strict_tracking_safety'] is True
    assert guard['tracking_max_age_s'] == 2.5
    assert guard['footprint_padding'] == 0.20
    _, planner = process(commands, 'planner_server')
    assert planner['GridBased']['cost_penalty'] == 3.0
    _, trace = process(commands, 'drive_trace')
    assert trace['directory'] == str(tmp_path / 'sessions' / 'drive_trace')
    assert Path(trace['session']).parent == tmp_path / 'sessions'
    _, navigator = process(commands, 'bt_navigator')
    _, controller = process(commands, 'controller_server')
    assert controller['FollowPath']['desired_linear_vel'] == 0.35
    assert controller['FollowPath']['regulated_linear_scaling_min_speed'] == 0.25
    assert navigator['default_server_timeout'] == 1000
    assert navigator['bt_loop_duration'] == 50
    names = [Path(cmd[0]).name for cmd, _ in commands]
    assert names.index('rgbd_odometry') < names.index('cp') < names.index('rtabmap')
    assert not any(cmd[:3] == ['ros2', 'bag', 'record'] for cmd, _ in commands)
    assert ('pixhawk_command_adapter' in names) == (mode == 'drive')
    if mode == 'drive':
        _, adapter = process(commands, 'pixhawk_command_adapter')
        assert adapter['cmd_vel_topic'] == '/cmd_vel_guarded'
        assert adapter['require_scan'] is False
        assert adapter['maximum_speed_m_s'] == 0.4
        assert adapter['hold_on_fault'] is True


@pytest.mark.parametrize(
    'change,expected',
    [
        ('static', True),
        ('route', False),
        ('rolling', False),
        ('updates', False),
        ('perception', False),
        ('filter', False),
    ],
)
def test_heuristic_cache_only_for_fixed_map(change, expected):
    params = yaml.safe_load((ROOT / 'config/nav2_rtab.yaml').read_text())
    costmap = params['global_costmap']['global_costmap']['ros__parameters']
    if change == 'rolling':
        costmap['rolling_window'] = True
    elif change == 'updates':
        costmap['static_layer']['subscribe_to_updates'] = True
    elif change == 'perception':
        costmap['plugins'].append('obstacles')
        costmap['obstacles'] = {'plugin': 'nav2_costmap_2d::ObstacleLayer'}
    elif change == 'filter':
        costmap['filters'] = ['keepout']
    launch = runpy.run_path(str(ROOT / 'launch/navigation.launch.py'))
    launch['cache_static_planner_heuristic'](params, change == 'route')
    assert (
        params['planner_server']['ros__parameters']['GridBased']['cache_obstacle_heuristic']
        is expected
    )


@pytest.mark.parametrize('odometry', ['camera', 'fused'])
def test_external_sensors_and_camera_tf_are_not_started_twice(tmp_path, odometry):
    result, commands = expand(
        tmp_path, external_sensors=True, local_odometry=odometry
    )
    assert result == 0
    names = {Path(cmd[0]).name for cmd, _ in commands}
    assert not names.intersection(
        {
            'mavros_node',
            'realsense2_camera_node',
            'static_transform_publisher',
            'imu_filter_madgwick_node',
            'rplidar_composition',
            'rplidar_node',
        }
    )
    telemetry = [(cmd, params) for cmd, params in commands
                 if Path(cmd[0]).name == 'imu_telemetry']
    # Fusion requests RAW_IMU (27); external MAVROS owns ATTITUDE_QUATERNION (31).
    # Neither the external MAVROS driver nor its attitude requester is duplicated.
    assert len(telemetry) == (1 if odometry == 'fused' else 0)
    if telemetry:
        assert telemetry[0][1]['message_id'] == 27
        assert telemetry[0][1]['imu_topic'] == '/mavros/imu/data_raw'
    assert {'rgbd_odometry', 'rtabmap', 'rviz2'} <= names


def test_failed_copy_never_starts_rtab(tmp_path):
    _, commands = expand(tmp_path, copy_result=1)
    assert not any(Path(cmd[0]).name == 'rtabmap' for cmd, _ in commands)


def test_legacy_map_search_limits_do_not_abort_distant_goals(tmp_path):
    params = yaml.safe_load((ROOT / 'config/nav2_rtab.yaml').read_text())
    planner = params['planner_server']['ros__parameters']['GridBased']
    planner['max_iterations'] = 1000000
    planner['max_planning_time'] = 5.0
    legacy = tmp_path / 'legacy_nav2.yaml'
    legacy.write_text(yaml.safe_dump(params))
    result, commands = expand(tmp_path, mode='drive', params_file=str(legacy))
    assert result == 0
    _, effective = process(commands, 'planner_server')
    effective = effective['GridBased']
    assert effective['max_iterations'] == -1
    assert effective['max_planning_time'] == 1000000000.0
    # Search budget must not loosen geometry/collision constraints or goal tolerance.
    for key in (
        'minimum_turning_radius', 'allow_unknown', 'motion_model_for_search',
        'tolerance', 'max_on_approach_iterations', 'downsample_costmap',
    ):
        assert effective[key] == planner[key]
    _, controller = process(commands, 'controller_server')
    assert controller['FollowPath']['desired_linear_vel'] == 0.35


def test_search_budget_preserves_other_planner_plugins():
    params = {'planner_server': {'ros__parameters': {
        'planner_plugins': ['Other'],
        'Other': {'plugin': 'custom/Planner', 'max_iterations': 123},
    }}}
    launch = runpy.run_path(str(ROOT / 'launch/navigation.launch.py'))
    launch['allow_long_planner_search'](params)
    assert params['planner_server']['ros__parameters']['Other']['max_iterations'] == 123


def test_old_map_uses_wider_plan_with_corrective_steering_margin(tmp_path):
    params = yaml.safe_load((ROOT / 'config/nav2_rtab.yaml').read_text())
    planner = params['planner_server']['ros__parameters']['GridBased']
    planner['minimum_turning_radius'] = 1.7
    planner['analytic_expansion_max_length'] = 8.5
    params['pixhawk_command_adapter'] = {'ros__parameters': {'minimum_turning_radius_m': 2.5}}
    old = tmp_path / 'old_geometry.yaml'
    old.write_text(yaml.safe_dump(params))
    result, commands = expand(tmp_path, mode='drive', params_file=str(old))
    assert result == 0
    _, effective = process(commands, 'planner_server')
    assert effective['GridBased']['minimum_turning_radius'] == 2.0
    assert effective['GridBased']['analytic_expansion_max_length'] == 10.0
    _, adapter = process(commands, 'pixhawk_command_adapter')
    assert adapter['minimum_turning_radius_m'] == 1.7
    from rover_navigation.pixhawk_command_adapter_node import bounded_motion
    for v in (0.25, 0.35, 0.4):
        # Planned curvature passes unchanged; feedback may request a tighter turn.
        assert bounded_motion(v, v / 2.0, 0.4, 1.7) == (v, v / 2.0)
        assert bounded_motion(v, 1.0, 0.4, 1.7) == (v, v / 1.7)


def write_startup_fixture(tmp_path):
    from rover_navigation.startup_pose import map_identity
    source = tmp_path / 'map'
    source.mkdir()
    (source / 'rtabmap.db').write_bytes(b'launch wiring fixture')
    # Match expand()'s deterministic map metadata.
    (source / 'rtabmap.yaml').write_text(yaml.safe_dump(
        {'image': 'rtabmap.pgm', 'origin': [0.0, 0.0, 0.0], 'resolution': 0.05}
    ))
    (source / 'startup_pose.yaml').write_text(yaml.safe_dump({
        'enabled': True, 'frame_id': 'map', 'base_frame_id': 'base_link',
        'source': map_identity(source / 'rtabmap.db', source / 'rtabmap.yaml'),
        'pose': [1, 2, 0, 0, 0, -0.8],
    }))


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_saved_startup_hint_goes_only_to_rtab_localization(tmp_path, enabled):
    write_startup_fixture(tmp_path)
    result, commands = expand(tmp_path, use_startup_pose=enabled)
    assert result == 0
    _, rtab = process(commands, 'rtabmap')
    assert rtab['initial_pose'] == ('1 2 0 0 0 -0.8' if enabled == 'true' else '')
    assert rtab['pub_loc_pose_only_when_localizing'] is True
    _, odom = process(commands, 'rgbd_odometry')
    assert not odom.get('initial_pose')
    _, guard = process(commands, 'localization_guard')
    assert guard['native_localization_only'] is True
    # No initialpose publisher or readiness override is added.
    assert not any('topic' in cmd and 'pub' in cmd for cmd, _ in commands)


@pytest.mark.parametrize('seeded', [True, False])
def test_memory_startup_success_or_cold_fallback(tmp_path, seeded):
    write_startup_fixture(tmp_path)
    result, commands = expand(tmp_path, memory_seeded=seeded)
    assert result == 0
    cmd, _ = process(commands, 'rtabmap')
    tokens = [token for part in cmd for token in shlex.split(part)]

    def last(name):
        return tokens[max(i for i, token in enumerate(tokens) if token == name) + 1]

    assert last('--Mem/InitWMWithAllNodes') == ('false' if seeded else 'true')
    assert last('--Rtabmap/MemoryThr') == ('600' if seeded else '0')
    assert last('--Kp/FlannIndexSaved') == ('false' if seeded else 'true')
    assert last('--Mem/LoadVisualLocalFeaturesOnInit') == 'false'
    _, odom = process(commands, 'rgbd_odometry')
    assert odom['always_process_most_recent_frame'] is True


def test_failed_memory_preparation_never_launches_rtab_with_partial_checkpoint(tmp_path):
    write_startup_fixture(tmp_path)
    _, commands = expand(tmp_path, memory_result=1)
    assert any(cmd[:3] == ['python3', '-m', 'rover_navigation.localization_memory']
               for cmd, _ in commands)
    assert not any(Path(cmd[0]).name == 'rtabmap' for cmd, _ in commands)


def test_full_mode_restores_checkpoint_before_starting_rtab(tmp_path):
    write_startup_fixture(tmp_path)
    _, commands = expand(tmp_path)
    _, params = process(commands, 'rtabmap')
    Path(params['database_path']).with_suffix('.global_graph.json').write_text('{}')
    result, commands = expand(tmp_path, localization_memory='full', memory_seeded=False)
    assert result == 0
    assert not any(cmd[0] == 'cp' for cmd, _ in commands)
    prep = next(cmd for cmd, _ in commands if cmd[:3] == [
        'python3', '-m', 'rover_navigation.localization_memory'])
    assert prep[prep.index('--mode') + 1] == 'full'
    cmd, _ = process(commands, 'rtabmap')
    tokens = [token for part in cmd for token in shlex.split(part)]
    assert tokens[tokens.index('--Mem/LoadVisualLocalFeaturesOnInit') + 1] == 'false'
    assert tokens[tokens.index('--Kp/ByteToFloat') + 1] == 'false'


def test_full_checkpoint_reuse_is_not_overridden_after_preparation(tmp_path):
    import sqlite3

    _, commands = expand(tmp_path, localization_memory='full', memory_seeded=False)
    _, params = process(commands, 'rtabmap')
    database = Path(params['database_path'])
    database.unlink()
    with sqlite3.connect(database) as db:
        db.executescript('''
            CREATE TABLE Node(id INTEGER, weight INTEGER, time_enter TEXT);
            CREATE TABLE Word(id INTEGER, time_enter TEXT);
            CREATE TABLE Info(dictionary_size INTEGER, time_enter TEXT);
            INSERT INTO Node VALUES(1, 0, '2026-10-22');
            INSERT INTO Word VALUES(1, '2026-10-22');
            INSERT INTO Info VALUES(1, '2026-10-22');
        ''')
    _, commands = expand(tmp_path, localization_memory='full', memory_seeded=False)
    cmd, _ = process(commands, 'rtabmap')
    tokens = [token for part in cmd for token in shlex.split(part)]
    flags = [tokens[i + 1] for i, token in enumerate(tokens)
             if token == '--Mem/InitWMWithAllNodes']
    assert flags and flags[-1] == 'false'
    assert not any(cmd[0] == 'cp' for cmd, _ in commands)


def test_per_map_yaml_without_bt_timing_uses_jetson_defaults(tmp_path):
    params = yaml.safe_load((ROOT / 'config/nav2_rtab.yaml').read_text())
    bt = params['bt_navigator']['ros__parameters']
    bt.pop('default_server_timeout')
    bt.pop('bt_loop_duration')
    filename = tmp_path / 'old_nav2.yaml'
    filename.write_text(yaml.safe_dump(params))
    result, commands = expand(tmp_path, params_file=str(filename))
    assert result == 0
    _, navigator = process(commands, 'bt_navigator')
    assert navigator['default_server_timeout'] == 1000
    assert navigator['bt_loop_duration'] == 50


def test_next_launch_reuses_private_database_without_copy(tmp_path):
    result, commands = expand(tmp_path)
    assert result == 0
    _, first = process(commands, 'rtabmap')
    database = Path(first['database_path'])
    database.write_bytes(b'private state saved by RTAB')
    result, commands = expand(tmp_path)
    assert result == 0
    assert not any(cmd[0] == 'cp' for cmd, _ in commands)
    _, second = process(commands, 'rtabmap')
    assert second['database_path'] == str(database)
    assert database.read_bytes() == b'private state saved by RTAB'
    assert (tmp_path / 'map/rtabmap.db').read_bytes() == b'launch wiring fixture'


def test_explicit_source_db_and_opt_in_recording(tmp_path):
    result, commands = expand(tmp_path, copy_database='false', record_bag='true',
                              localization_memory='full')
    assert result == 0
    assert not any(cmd[0] == 'cp' for cmd, _ in commands)
    _, rtab = process(commands, 'rtabmap')
    assert rtab['database_path'] == str(tmp_path / 'map/rtabmap.db')
    assert any(cmd[:3] == ['ros2', 'bag', 'record'] for cmd, _ in commands)


def test_standard_pose_tool_can_operate_before_localization():
    config = yaml.safe_load((ROOT / 'config/navigation.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert manager['Views']['Current']['Target Frame'] == 'map'
    pose = [
        tool for tool in manager['Tools'] if tool['Class'] == 'rviz_default_plugins/SetInitialPose'
    ]
    assert len(pose) == 1 and pose[0]['Topic'] == '/initialpose'
