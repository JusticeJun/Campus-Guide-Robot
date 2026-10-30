"""Actuator-free mapping with processed RGB-D and a pinned per-map runtime."""

from pathlib import Path
import shutil
import yaml

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
from rover_navigation.multisensor_runtime import (
    fused_nodes, scan_arguments, registration_arguments,
)


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    resume = value('continue').lower()
    viz = value('rtabmap_viz').lower()
    if resume not in ('true', 'false') or viz not in ('true', 'false'):
        raise ValueError('continue and rtabmap_viz must be true or false')
    switches = {}
    for name in ('start_mavros', 'start_camera', 'start_lidar'):
        flag = value(name).lower()
        if flag not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        switches[name] = flag == 'true'
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
    selected_mode = value('sensors')
    sensor_mode = runtime.get('sensor_mode', 'rgbd') if selected_mode == 'auto' else selected_mode
    if sensor_mode not in ('rgbd', 'rgbd_lidar'):
        raise ValueError('sensors must be auto, rgbd or rgbd_lidar')
    if resume == 'true' and sensor_mode != runtime.get('sensor_mode', 'rgbd'):
        raise ValueError('Continue must keep the original sensor mode; use a new map directory')
    runtime['sensor_mode'] = sensor_mode
    fused = sensor_mode == 'rgbd_lidar'
    profile = runtime['profile']
    config = runtime['rtabmap']
    if resume == 'false':
        directory.mkdir(parents=True, exist_ok=True)
        if fused:
            pinned.write_text(yaml.safe_dump(runtime, sort_keys=False))
        else:
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
        # Keep visual odometry responsive while the map builds its dictionary,
        # optimizes loops or compresses new RGB-D observations for storage.
        # odometry_node() is separate and remains at normal priority.
        'launch_prefix': 'nice -n 5',
        **{key: profile[key] for key in ('rgb_topic', 'depth_topic', 'camera_info_topic')},
    }
    arguments['args'] = config.get('args', '') + (
        ' --Mem/IncrementalMemory true --Mem/InitWMWithAllNodes true'
        ' --Rtabmap/StartNewMapOnLoopClosure ' + resume
    )
    if fused:
        arguments.update(scan_arguments(config))
        arguments['args'] += registration_arguments()
    sensors = []
    share = Path(get_package_share_directory('rover_navigation'))
    for switch, filename, settings in (
        ('start_mavros', 'mavros.launch.py', {'fcu_url': value('fcu_url')}),
        # The just-created/preserved snapshot owns both camera settings and TF.
        ('start_camera', 'camera.launch.py', {'runtime_file': str(pinned)}),
        ('start_lidar', 'lidar.launch.py',
         {'serial_port': value('lidar_port'), 'yaw': value('lidar_yaw')}),
    ):
        if switches[switch]:
            sensors.append(GroupAction([
                *runtime_environment(),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
                    launch_arguments=settings.items(),
                ),
            ], forwarding=False))
    return [
        LogInfo(msg=f'Mapping database: {database}; runtime snapshot: {pinned}'),
        LogInfo(
            msg=f'Embedded sensors: {switches}. Sensors not started here must run separately. '
            'Camera/lidar launch owns its mounting TF. No vehicle controller is launched.'
        ),
        *sensors,
        *(fused_nodes(profile, config) if fused else [odometry_node(profile, config)]),
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
            DeclareLaunchArgument('start_mavros', default_value='false'),
            DeclareLaunchArgument('start_camera', default_value='false'),
            DeclareLaunchArgument('start_lidar', default_value='false'),
            DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyACM0:115200'),
            DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
            DeclareLaunchArgument('sensors', default_value='auto',
                                  description=(
                                      'auto: pinned mode; rgbd_lidar: camera + scan + EKF')),
            DeclareLaunchArgument('rtabmap_viz', default_value='true'),
            DeclareLaunchArgument('sigterm_timeout', default_value='120'),
            OpaqueFunction(function=setup),
        ]
    )
