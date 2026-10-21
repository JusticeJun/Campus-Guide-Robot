"""Expand mapping launch without hardware; protect maps and shared transport."""

import os
from pathlib import Path
import shlex
from unittest.mock import patch

import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_local import ExecuteLocal
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]


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
