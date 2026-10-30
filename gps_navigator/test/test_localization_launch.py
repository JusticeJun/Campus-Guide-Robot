"""Expand real launch files without processes; exercise DB exit and RGB-D wiring."""

import os
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


def expand(tmp_path, *, copy_result=0, **overrides):
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
        'start_camera': 'true',
        'database_cache_root': str(tmp_path / 'cache'),
        'start_mavros': 'true',
        'rviz': 'true',
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


def test_external_sensors_and_camera_tf_are_not_started_twice(tmp_path):
    result, commands = expand(
        tmp_path, start_mavros='false', start_camera='false', start_camera_tf='false'
    )
    assert result == 0
    names = {Path(cmd[0]).name for cmd, _ in commands}
    assert not names.intersection(
        {
            'mavros_node',
            'realsense2_camera_node',
            'static_transform_publisher',
            'imu_telemetry',
            'imu_filter_madgwick_node',
        }
    )
    assert {'rgbd_odometry', 'rtabmap', 'rviz2'} <= names


def test_failed_copy_never_starts_rtab(tmp_path):
    _, commands = expand(tmp_path, copy_result=1)
    assert not any(Path(cmd[0]).name == 'rtabmap' for cmd, _ in commands)


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
    result, commands = expand(tmp_path, copy_database='false', record_bag='true')
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
