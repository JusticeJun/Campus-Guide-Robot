"""MAVROS plus a session-only attitude stream request for camera-rate IMU."""

from rover_navigation.rtab_runtime import runtime_environment
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            *runtime_environment(),
            DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyACM0:115200'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    str(Path(get_package_share_directory('mavros')) / 'launch/apm.launch')
                ),
                launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
            ),
            Node(package='rover_navigation', executable='imu_telemetry', output='screen'),
        ]
    )
