"""One odometry input/transport configuration for mapping and localization."""

import shlex
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def runtime_environment():
    """Defaults for the on-Jetson stack; explicit caller environment wins."""
    config = Path(get_package_share_directory('rover_navigation')) / 'config/cyclonedds.xml'
    defaults = {
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': config.as_uri(),
        'ROS_LOCALHOST_ONLY': '1',
        'OPENCV_FOR_THREADS_NUM': '2',
        'OMP_NUM_THREADS': '2',
        'OPENBLAS_NUM_THREADS': '1',
    }
    return [
        SetEnvironmentVariable(name, EnvironmentVariable(name, default_value=value))
        for name, value in defaults.items()
    ]


def odometry_node(profile, rtab_config):
    def boolean(name, default):
        setting = str(rtab_config.get(name, default)).lower()
        if setting not in ('true', 'false'):
            raise ValueError(f'rtabmap.{name} must be true or false')
        return setting == 'true'

    qos = int(rtab_config.get('qos', 2))
    odom_parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        # Processed RGB-D is also the map/mesh archive; retain its RGB channels.
        'keep_color': True,
        'wait_for_transform': float(rtab_config.get('wait_for_transform', 0.2)),
        'wait_imu_to_init': boolean('wait_imu_to_init', True),
        'always_check_imu_tf': boolean('always_check_imu_tf', True),
        'always_process_most_recent_frame': boolean('odom_always_process_most_recent_frame', True),
        'approx_sync': boolean('approx_sync', True),
        'approx_sync_max_interval': float(rtab_config.get('approx_sync_max_interval', 0.02)),
        'topic_queue_size': int(rtab_config.get('topic_queue_size', 10)),
        'sync_queue_size': int(rtab_config.get('sync_queue_size', 10)),
        'qos': int(rtab_config.get('qos_image', qos)),
        'qos_camera_info': int(rtab_config.get('qos_camera_info', qos)),
        'qos_imu': int(rtab_config.get('qos_imu', qos)),
    }
    return Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[odom_parameters],
        arguments=shlex.split(rtab_config.get('args', ''))
        + shlex.split(rtab_config.get('odom_args', '')),
        remappings=[
            ('rgb/image', profile['rgb_topic']),
            ('depth/image', profile['depth_topic']),
            ('rgb/camera_info', profile['camera_info_topic']),
            ('imu', rtab_config.get('imu_topic') or '/navigation/unused/imu'),
            ('odom', '/odom'),
        ],
    )
