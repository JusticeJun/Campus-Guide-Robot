"""Start the camera with the same profile as mapping and navigation."""

from rover_navigation.rtab_runtime import runtime_environment
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rover_navigation.map_session import read_mapping


def setup(context):
    runtime = read_mapping(LaunchConfiguration('runtime_file').perform(context))
    publish_tf = LaunchConfiguration('publish_camera_tf').perform(context).lower()
    if publish_tf not in ('true', 'false'):
        raise ValueError('publish_camera_tf must be true or false')
    actions = [
        GroupAction(
            [
                *runtime_environment(),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(
                            Path(get_package_share_directory('realsense2_camera'))
                            / 'launch/rs_launch.py'
                        )
                    ),
                    launch_arguments=runtime['camera'].items(),
                ),
            ],
            forwarding=False,
        )
    ]
    if publish_tf == 'true':
        x, y, z, roll, pitch, yaw = runtime['profile']['base_to_camera']
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_camera',
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                    '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
                ],
            )
        )
    # Gravity orientation from camera gyro/accel; camera driver owns sensor TFs.
    if 'imu_filter' in runtime:
        config = runtime['imu_filter']
        actions.append(
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='camera_imu_filter',
                output='screen',
                parameters=[
                    {key: config[key] for key in ('use_mag', 'world_frame', 'publish_tf')}
                ],
                remappings=[
                    ('imu/data_raw', config['raw_topic']),
                    ('imu/data', config['output_topic']),
                ],
            )
        )
    return actions


def generate_launch_description():
    default = str(Path(get_package_share_directory(
        'rover_navigation')) / 'config/rtab_runtime.yaml')
    return LaunchDescription(
        [
            *runtime_environment(),
            DeclareLaunchArgument('runtime_file', default_value=default),
            DeclareLaunchArgument('publish_camera_tf', default_value='true'),
            OpaqueFunction(function=setup),
        ]
    )
