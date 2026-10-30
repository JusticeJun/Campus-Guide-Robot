"""Check camera TF ownership and per-map mounting without starting hardware."""

from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_local import ExecuteLocal
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def expand(**settings):
    commands = []

    def capture(action, context):
        commands.append([perform_substitutions(context, part) for part in action.cmd])
        return []

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / 'launch/camera.launch.py')),
            launch_arguments=settings.items(),
        ),
    ]))
    with patch.object(ExecuteLocal, 'execute', capture):
        result = service.run()
    assert result == 0
    return commands


@pytest.mark.parametrize('pinned', [False, True])
def test_camera_includes_exactly_one_mounting_tf(tmp_path, pinned):
    runtime = yaml.safe_load((ROOT / 'config/rtab_runtime.yaml').read_text())
    settings = {}
    if pinned:
        runtime['profile']['base_to_camera'] = [0.4, -0.03, 0.25, 0.01, 0.02, -0.1]
        snapshot = tmp_path / 'localization.yaml'
        snapshot.write_text(yaml.safe_dump(runtime))
        settings['runtime_file'] = str(snapshot)
    commands = expand(**settings)
    transforms = [cmd for cmd in commands if Path(cmd[0]).name == 'static_transform_publisher']
    assert len(transforms) == 1
    cmd = transforms[0]
    for flag, expected in zip(
        ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'),
        runtime['profile']['base_to_camera'],
    ):
        assert float(cmd[cmd.index(flag) + 1]) == expected
    assert cmd[cmd.index('--frame-id') + 1] == 'base_link'
    assert cmd[cmd.index('--child-frame-id') + 1] == 'camera_link'
    assert sum(Path(c[0]).name == 'realsense2_camera_node' for c in commands) == 1


def test_embedded_camera_can_leave_tf_to_navigation():
    commands = expand(publish_camera_tf='false')
    assert not any(Path(cmd[0]).name == 'static_transform_publisher' for cmd in commands)
    assert any(Path(cmd[0]).name == 'realsense2_camera_node' for cmd in commands)
