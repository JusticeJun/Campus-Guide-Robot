"""Actuator-free mapping with processed RGB-D and a pinned per-map runtime."""

from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

from rover_navigation.map_session import read_mapping
from rover_navigation.rtab_runtime import odometry_node, runtime_environment


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    resume = value('continue').lower()
    viz = value('rtabmap_viz').lower()
    if resume not in ('true', 'false') or viz not in ('true', 'false'):
        raise ValueError('continue and rtabmap_viz must be true or false')
    directory = Path(value('map_dir')).expanduser().resolve()
    database = directory / 'rtabmap.db'
    pinned = directory / 'localization.yaml'
    if resume == 'false' and (database.exists() or pinned.exists()):
        raise ValueError(
            'New mapping requires an unused map directory; use continue:=true for an existing map'
        )
    if resume == 'true' and (not database.is_file() or not pinned.is_file()):
        raise ValueError('Continue requires rtabmap.db and its localization.yaml snapshot')
    source = pinned if resume == 'true' else Path(value('runtime_file')).expanduser().resolve()
    runtime = read_mapping(source)
    profile = runtime['profile']
    config = runtime['rtabmap']
    if resume == 'false':
        directory.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, pinned)
    arguments = {
        **config,
        'database_path': str(database),
        'frame_id': 'base_link',
        'vo_frame_id': 'odom',
        'map_frame_id': 'map',
        'odom_topic': '/odom',
        'odom_frame_id': '',
        'namespace': 'rtabmap',
        'localization': 'false',
        'visual_odometry': 'false',
        'subscribe_rgbd': 'true',
        'rgbd_sync': 'false',
        'rgbd_topic': '/rtabmap/odom_rgbd_image',
        'map_topic': '/rtabmap/map',
        'gps_topic': '/navigation/unused/gps/fix',
        'rtabmap_viz': viz,
        'rviz': 'false',
        **{key: profile[key] for key in ('rgb_topic', 'depth_topic', 'camera_info_topic')},
    }
    arguments['args'] = config.get('args', '') + (
        ' --Mem/IncrementalMemory true --Mem/InitWMWithAllNodes true'
        ' --Rtabmap/StartNewMapOnLoopClosure ' + resume
    )
    return [
        LogInfo(msg=f'Mapping database: {database}; runtime snapshot: {pinned}'),
        LogInfo(
            msg='External camera, MAVROS and base_link->camera_link TF required. '
            'No vehicle controller is launched.'
        ),
        odometry_node(profile, config),
        GroupAction(
            [
                *runtime_environment(),
                SetRemap(src='initialpose', dst='/initialpose'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(
                            Path(get_package_share_directory('rtabmap_launch'))
                            / 'launch/rtabmap.launch.py'
                        )
                    ),
                    launch_arguments=arguments.items(),
                ),
            ],
            forwarding=False,
        ),
    ]


def generate_launch_description():
    default = str(Path(get_package_share_directory(
        'rover_navigation')) / 'config/rtab_runtime.yaml')
    return LaunchDescription(
        [
            *runtime_environment(),
            DeclareLaunchArgument(
                'map_dir', description='New map directory, or an existing map with continue:=true'
            ),
            DeclareLaunchArgument('runtime_file', default_value=default),
            DeclareLaunchArgument('continue', default_value='false'),
            DeclareLaunchArgument('rtabmap_viz', default_value='true'),
            DeclareLaunchArgument('sigterm_timeout', default_value='120'),
            OpaqueFunction(function=setup),
        ]
    )
