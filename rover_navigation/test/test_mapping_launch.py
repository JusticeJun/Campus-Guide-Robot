"""Expand mapping launch without hardware; protect maps and shared transport."""

import os
from pathlib import Path
import shlex
from unittest.mock import patch

import yaml
import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_local import ExecuteLocal
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('resume', [False, True])
def test_integrated_mapping_starts_each_sensor_and_tf_once(tmp_path, resume):
    directory = tmp_path / 'integrated'
    settings = {'sensors': 'rgbd_lidar'}
    if resume:
        directory.mkdir()
        (directory / 'rtabmap.db').write_bytes(b'preserved mapping fixture')
        runtime = yaml.safe_load((ROOT / 'config/rtab_runtime.yaml').read_text())
        runtime['sensor_mode'] = 'rgbd_lidar'
        runtime['profile']['base_to_camera'][0] = 0.42
        (directory / 'localization.yaml').write_text(yaml.safe_dump(runtime))
        settings = {'continue': 'true'}
    result, commands = expand(directory, start_mavros='true', start_camera='true',
                              start_lidar='true', lidar_port='/dev/test_lidar', **settings)
    assert result == 0
    names = [Path(c[0]).name for c in commands]
    for name in ('mavros_node', 'realsense2_camera_node', 'rplidar_node',
                 'imu_filter_madgwick_node', 'front_scan'):
        assert names.count(name) == 1
    transforms = [c for c in commands if Path(c[0]).name == 'static_transform_publisher']
    assert len(transforms) == 2
    children = {c[c.index('--child-frame-id') + 1]: c for c in transforms}
    assert set(children) == {'camera_link', 'laser'}
    camera = children['camera_link']
    runtime = yaml.safe_load((directory / 'localization.yaml').read_text())
    assert float(camera[camera.index('--x') + 1]) == runtime['profile']['base_to_camera'][0]
    assert 'pixhawk_command_adapter' not in names


def expand(directory, **overrides):
    commands = []

    def capture(action, context):
        assert context.environment['RMW_IMPLEMENTATION'] == os.environ.get(
            'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'
        )
        assert context.environment.get('CYCLONEDDS_URI')
        commands.append([perform_substitutions(context, part) for part in action.cmd])
        return []

    service = LaunchService()
    service.include_launch_description(
        LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(ROOT / 'launch/mapping.launch.py')),
                    launch_arguments={
                        'map_dir': str(directory),
                        'rtabmap_viz': 'false',
                        **overrides,
                    }.items(),
                )
            ]
        )
    )
    with patch.object(ExecuteLocal, 'execute', capture):
        result = service.run()
    return result, commands


def test_mapping_preserves_raw_images_and_uses_same_processed_path(tmp_path):
    directory = tmp_path / 'new'
    result, commands = expand(directory)
    assert result == 0
    assert (directory / 'localization.yaml').read_bytes() == (
        ROOT / 'config/rtab_runtime.yaml'
    ).read_bytes()
    names = [Path(c[0]).name for c in commands]
    assert sorted(names) == ['rgbd_odometry', 'rgbd_relay', 'rtabmap']
    rtab = next(c for c in commands if Path(c[0]).name == 'rtabmap')
    assert 'rgbd_image:=/rtabmap/odom_rgbd_image_relay' in rtab
    params = {}
    for i, arg in enumerate(rtab):
        if arg == '--params-file':
            for entry in yaml.full_load(Path(rtab[i + 1]).read_text()).values():
                params.update(entry.get('ros__parameters', {}))
    assert params['Mem/IncrementalMemory'] == 'true'
    assert params['subscribe_rgbd'] is True
    assert params['qos_image'] == 1 and params['qos_imu'] == 2
    assert params['database_path'] == str(directory / 'rtabmap.db')


def test_existing_map_requires_explicit_continue_and_preserves_snapshot(tmp_path):
    directory = tmp_path / 'map'
    directory.mkdir()
    database = directory / 'rtabmap.db'
    database.write_bytes(b'untouched map')
    source = yaml.safe_load((ROOT / 'config/rtab_runtime.yaml').read_text())
    source['rtabmap']['args'] += ' --Rtabmap/DetectionRate 1.5'
    snapshot = directory / 'localization.yaml'
    snapshot.write_text(yaml.safe_dump(source))
    before = snapshot.read_bytes()
    result, commands = expand(directory)
    assert result != 0 and not commands
    result, commands = expand(directory, **{'continue': 'true'})
    assert result == 0
    rtab = next(c for c in commands if Path(c[0]).name == 'rtabmap')
    tokens = [t for part in rtab for t in shlex.split(part)]
    assert tokens[tokens.index('--Rtabmap/StartNewMapOnLoopClosure') + 1] == 'true'
    assert '--Rtabmap/DetectionRate 1.5' in ' '.join(rtab)
    assert database.read_bytes() == b'untouched map'
    assert snapshot.read_bytes() == before


def test_continue_without_map_is_rejected(tmp_path):
    result, commands = expand(tmp_path / 'missing', **{'continue': 'true'})
    assert result != 0 and not commands


def test_lidar_mapping_pins_mode_and_continues_with_same_sensor_graph(tmp_path):
    result, commands = expand(tmp_path / 'scan_map', sensors='rgbd_lidar')
    assert result == 0
    names = [Path(c[0]).name for c in commands]
    assert 'icp_odometry' in names and 'ekf_node' in names and 'fusion_inputs' in names
    assert 'pixhawk_command_adapter' not in names and 'external_navigation' not in names
    directory = tmp_path / 'scan_map'
    snapshot = yaml.safe_load((directory / 'localization.yaml').read_text())
    assert snapshot['sensor_mode'] == 'rgbd_lidar'
    (directory / 'rtabmap.db').write_bytes(b'scan db fixture')
    result, commands = expand(directory, **{'continue': 'true'})
    assert result == 0
    rtab = next(c for c in commands if Path(c[0]).name == 'rtabmap')
    assert 'scan:=/scan_front' in rtab
    assert '--Reg/Strategy 2' in ' '.join(rtab)
    result, commands = expand(directory, **{'continue': 'true', 'sensors': 'rgbd'})
    assert result != 0 and not commands


def test_new_and_continued_mapping_open_viz_with_matching_rgbd_source(tmp_path):
    for sensor_mode, image in (
        ('rgbd', '/rtabmap/odom_rgbd_image_relay'),
        ('rgbd_lidar', '/vo/odom_rgbd_image_relay'),
    ):
        directory = tmp_path / sensor_mode
        result, commands = expand(directory, sensors=sensor_mode, rtabmap_viz='true')
        assert result == 0
        viz = [cmd for cmd in commands if Path(cmd[0]).name == 'rtabmap_viz']
        assert len(viz) == 1
        assert f'rgbd_image:={image}' in viz[0]
        (directory / 'rtabmap.db').write_bytes(b'unchanged map fixture')
        result, commands = expand(directory, **{'continue': 'true', 'rtabmap_viz': 'true'})
        assert result == 0
        viz = [cmd for cmd in commands if Path(cmd[0]).name == 'rtabmap_viz']
        assert len(viz) == 1
        assert f'rgbd_image:={image}' in viz[0]
