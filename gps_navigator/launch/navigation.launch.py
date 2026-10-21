"""Shared RTAB/Nav2 runtime for map goals and optional legacy route input."""

from datetime import datetime
import json
import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from nav2_common.launch import RewrittenYaml
import yaml

from gps_navigator.rtab_runtime import odometry_node, runtime_environment
from gps_navigator.map_session import read_mapping, select_session
from gps_navigator.localization_database import LocalizationDatabase, full_working_memory_saved
from gps_navigator.path_validation import shared_footprint
from gps_navigator.speed_policy import apply_speed_policy


def cache_static_planner_heuristic(params, needs_route):
    """Reuse same-goal search work only for our fixed exported occupancy map."""
    costmap = params.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
    layers = [
        costmap.get(name, {})
        for name in costmap.get('plugins', [])
        if name != 'route_layer' and costmap.get(name, {}).get('enabled', True)
    ]
    static = [layer for layer in layers if layer.get('plugin') == 'nav2_costmap_2d::StaticLayer']
    eligible = (
        not needs_route
        and costmap.get('rolling_window') is False
        and not costmap.get('filters')
        and len(static) == 1
        and static[0].get('map_topic') == '/map'
        and not static[0].get('subscribe_to_updates', False)
        and all(
            layer.get('plugin')
            in ('nav2_costmap_2d::StaticLayer', 'nav2_costmap_2d::InflationLayer')
            for layer in layers
        )
    )
    planner = params.get('planner_server', {}).get('ros__parameters', {})
    for name in planner.get('planner_plugins', []):
        plugin = planner.get(name, {})
        if plugin.get('plugin') == 'nav2_smac_planner/SmacPlannerHybrid':
            plugin['cache_obstacle_heuristic'] = eligible


def setup_navigation(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('mode')
    switches = {}
    for name in (
        'start_mavros',
        'start_camera',
        'start_camera_tf',
        'rviz',
        'record_bag',
        'copy_database',
        'reuse_database',
    ):
        if value(name).lower() not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        switches[name] = value(name).lower() == 'true'
    record_camera = value('record_camera').lower()
    if record_camera not in ('true', 'false'):
        raise ValueError('record_camera must be true or false')
    switches['record_bag'] = switches['record_bag'] or record_camera == 'true'
    policy = value('route_policy')
    if policy not in ('direct', 'graph'):
        raise ValueError('route_policy must be direct or graph')
    needs_route = bool(value('destination'))
    if policy == 'graph' and not needs_route:
        raise ValueError('graph policy requires the legacy destination input')
    profile, params_file, localization_file, profile_path = select_session(
        mode,
        map_dir=value('map_dir'),
        profile=value('profile'),
        database=value('database'),
        occupancy=value('map'),
        params_file=value('params_file'),
        localization_file=value('localization_file'),
        route_required=needs_route,
    )
    runtime = read_mapping(localization_file)
    if (
        needs_route
        and mode != 'planner'
        and policy == 'graph'
        and not profile.get('reviewed_edges')
    ):
        raise ValueError('graph policy needs reviewed_edges in profile')
    nav_params = yaml.safe_load(Path(params_file).read_text())
    # Also cover per-map YAML that omits these and would inherit Humble's
    # 10 ms tick / 20 ms action acknowledgement deadline under camera load.
    bt_params = nav_params['bt_navigator']['ros__parameters']
    bt_params.setdefault('bt_loop_duration', 50)
    bt_params.setdefault('default_server_timeout', 1000)
    turn_speed = apply_speed_policy(nav_params, value('turn_speed_m_s'))
    cache_static_planner_heuristic(nav_params, needs_route)
    # Apply to old per-map YAML too. Validation exercises the same guard as drive.
    nav_params.setdefault('localization_guard', {}).setdefault('ros__parameters', {}).update(
        {'strict_tracking_safety': True, 'tracking_max_age_s': 2.5}
    )
    footprint, footprint_padding = shared_footprint(nav_params)
    share = get_package_share_directory('gps_navigator')
    session = Path(value('log_root')).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    session.mkdir(parents=True)
    (session / 'profile.resolved.yaml').write_text(yaml.safe_dump(profile))
    (session / 'run.yaml').write_text(
        yaml.safe_dump(
            {
                'mode': mode,
                'route_policy': policy,
                'destination': value('destination'),
                'goal_input': 'legacy_route' if needs_route else 'nav2_action',
                **switches,
                'record_camera': record_camera == 'true',
                'speed_policy': {
                    'desired_m_s': 0.35,
                    'cap_m_s': 0.4,
                    'turn_floor_m_s': turn_speed,
                    'scaling_radius_m': 3.0,
                },
            }
        )
    )
    shutil.copyfile(params_file, session / 'nav2.input.yaml')
    shutil.copyfile(localization_file, session / 'localization.input.yaml')
    if not needs_route and mode != 'planner':
        # Preserve user plugin lists; Humble defaults come from its installed config.
        if 'plugin_lib_names' not in bt_params:
            defaults = read_mapping(
                os.path.join(
                    get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'
                )
            )
            bt_params['plugin_lib_names'] = defaults['bt_navigator']['ros__parameters'][
                'plugin_lib_names'
            ]
        if 'localization_recovery_bt' not in bt_params['plugin_lib_names']:
            bt_params['plugin_lib_names'].append('localization_recovery_bt')
    # Apply even with old per-map YAML and in legacy route/planner modes.
    params_file = str(session / 'nav2.resolved.yaml')
    Path(params_file).write_text(yaml.safe_dump(nav_params))
    rewrites = {
        'default_nav_through_poses_bt_xml': os.path.join(
            share, 'behavior_trees', 'navigate_through_poses_ackermann.xml'
        ),
        'default_nav_to_pose_bt_xml': os.path.join(
            share,
            'behavior_trees',
            (
                'navigate_to_pose_ackermann.xml'
                if needs_route
                else 'navigate_to_pose_localization.xml'
            ),
        ),
    }
    for name in ('local_costmap', 'global_costmap'):
        rewrites[f'{name}.{name}.ros__parameters.route_layer.enabled'] = (
            'true' if mode != 'planner' and needs_route else 'false'
        )
    params = RewrittenYaml(
        source_file=params_file, root_key='', convert_types=True, param_rewrites=rewrites
    )
    actions = [LogInfo(msg=f'Navigation session: {session}')]
    if mode != 'planner':
        actions.append(
            Node(
                package='gps_navigator',
                executable='drive_trace',
                name='drive_trace',
                output='screen',
                parameters=[
                    {
                        'directory': str(Path(value('log_root')).expanduser() / 'drive_trace'),
                        'session': str(session),
                    }
                ],
            )
        )
        if switches['start_mavros']:
            actions.append(
                GroupAction(
                    [
                        *runtime_environment(),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(share, 'launch', 'mavros.launch.py')
                            ),
                            launch_arguments={'fcu_url': value('fcu_url')}.items(),
                        ),
                    ],
                    forwarding=False,
                )
            )
        if switches['start_camera']:
            actions.append(
                GroupAction(
                    [
                        *runtime_environment(),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(share, 'launch', 'camera.launch.py')
                            ),
                            launch_arguments={'runtime_file': localization_file}.items(),
                        ),
                    ],
                    forwarding=False,
                )
            )
        topics = [
            '/rosout',
            '/map',
            '/navigation/route_map',
            '/gps_navigation/route',
            '/plan',
            '/global_costmap/costmap',
            '/local_costmap/costmap',
            '/local_plan',
            '/odom',
            '/rtabmap/odom_info',
            '/diagnostics',
            '/rtabmap/localization_pose',
            '/rtabmap/info',
            '/tf',
            '/tf_static',
            '/navigation/status',
            '/navigation/route_status',
            '/navigation/ready',
            '/navigation/route_active',
            '/cmd_vel',
            '/cmd_vel_guarded',
            '/mavros/setpoint_raw/local',
            '/mavros/state',
            '/mavros/global_position/global',
            '/mavros/global_position/compass_hdg',
            '/mavros/imu/data',
            '/cmd_vel_nav',
            '/parameter_events',
            '/lookahead_point',
            '/received_global_plan',
            '/speed_limit',
            # Record candidate measured motion and its estimator health so a VO
            # loss can be distinguished from commanded motion. These topics are
            # diagnostic inputs only; they do not become odometry or TF sources.
            '/mavros/local_position/odom',
            '/mavros/local_position/velocity_body',
            '/mavros/local_position/velocity_local',
            '/mavros/estimator_status',
            '/mavros/imu/data_raw',
            '/mavros/rc/out',
            '/navigate_to_pose/_action/status',
            '/navigate_to_pose/_action/feedback',
        ]
        imu_topic = runtime['rtabmap'].get('imu_topic')
        if imu_topic and imu_topic not in topics:
            topics.append(imu_topic)
        if record_camera == 'true':
            topics.extend(
                profile[key] for key in ('rgb_topic', 'depth_topic', 'camera_info_topic')
            )
        recorder = ExecuteProcess(
            output='log',
            cmd=[
                'ros2',
                'bag',
                'record',
                '--include-hidden-topics',
                '-o',
                str(session / 'bag'),
                *topics,
            ],
        )
        if switches['record_bag']:
            actions.extend(
                [
                    recorder,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=recorder,
                            on_exit=[
                                EmitEvent(event=Shutdown(reason='navigation recorder exited'))
                            ],
                        )
                    ),
                ]
            )
    actions.append(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': profile['map'], 'frame_id': 'map'}],
        )
    )
    lifecycle = ['map_server']
    if mode == 'planner':
        actions.append(
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[params],
            )
        )
        lifecycle.append('planner_server')
    else:
        # RTAB may update a database even in localization mode. Normally reuse
        # a private working DB; keep its writer lock until launch fully exits.
        database_lease = None
        if switches['copy_database'] and switches['reuse_database']:
            database_lease = LocalizationDatabase(
                profile['database'], runtime, value('database_cache_root')
            )
            context.extend_globals({'navigation_database_lease': database_lease})
        database = (
            session / 'localization.db' if switches['copy_database'] else Path(profile['database'])
        )
        if database_lease:
            database = database_lease.path
        (session / 'database.yaml').write_text(
            yaml.safe_dump(
                {
                    'source': profile['database'],
                    'runtime': str(database),
                    'reused': bool(database_lease and database_lease.ready()),
                    'source_writable_by_rtab': not switches['copy_database'],
                }
            )
        )
        if switches['start_camera_tf']:
            x, y, z, roll, pitch, yaw = profile['base_to_camera']
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_to_camera',
                    arguments=[
                        '--x',
                        str(x),
                        '--y',
                        str(y),
                        '--z',
                        str(z),
                        '--roll',
                        str(roll),
                        '--pitch',
                        str(pitch),
                        '--yaw',
                        str(yaw),
                        '--frame-id',
                        'base_link',
                        '--child-frame-id',
                        'camera_link',
                    ],
                )
            )
        # Odometry publishes the RGB-D frame it actually processed, with the
        # same stamp as /odom. Do not sync delayed /odom against the live 15 Hz
        # camera queue a second time (10 images retain only ~0.67 seconds).
        rtab_config = runtime['rtabmap']
        actions.append(odometry_node(profile, rtab_config))
        rtab_args = {
            **rtab_config,
            'database_path': str(database),
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'vo_frame_id': 'odom',
            'odom_topic': '/odom',
            'odom_frame_id': '',
            'namespace': 'rtabmap',
            'localization': 'true',
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'publish_tf_map': 'true',
            'publish_tf_odom': 'true',
            'visual_odometry': 'false',
            'subscribe_rgbd': 'true',
            'rgbd_sync': 'false',
            'rgbd_topic': '/rtabmap/odom_rgbd_image',
            'map_topic': '/rtabmap/map',
            'gps_topic': '/navigation/unused/gps/fix',
            **{key: profile[key] for key in ('rgb_topic', 'depth_topic', 'camera_info_topic')},
        }
        # Reuse RTAB's own index, without narrowing the set of searchable nodes.
        # A first/unfinished session still loads all nodes; the next clean full
        # checkpoint can restore the same coverage and its serialized index.
        restore_memory = (
            not switches['copy_database'] or (database_lease and database_lease.ready())
        ) and full_working_memory_saved(database)
        rtab_args['args'] = rtab_config.get('args', '') + ' --Kp/FlannIndexSaved true'
        if restore_memory:
            rtab_args['args'] += ' --Mem/InitWMWithAllNodes false'
        rtab_action = GroupAction(
            [
                *runtime_environment(),
                SetParameter(name='pub_loc_pose_only_when_localizing', value=True),
                # Nav2 already uses the exported occupancy via map_server.
                # Avoid loading every stored local grid into a second map cache.
                SetParameter(name='map_cache_loaded_on_init', value=False),
                SetRemap(src='initialpose', dst='/initialpose'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('rtabmap_launch'),
                            'launch',
                            'rtabmap.launch.py',
                        )
                    ),
                    launch_arguments=rtab_args.items(),
                ),
            ],
            forwarding=False,
        )
        # Copy outside the launch event loop. Sensors, odometry and RViz can
        # start while the private DB is prepared; RTAB waits for successful exit.
        database_copy = ExecuteProcess(
            cmd=[
                'cp',
                '--reflink=auto',
                profile['database'],
                str(database_lease.pending if database_lease else database),
            ],
            name='prepare_localization_database',
            output='screen',
        )

        def database_prepared(event, context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='localization database copy failed'))]
            if database_lease:
                try:
                    database_lease.finish_copy()
                except (OSError, ValueError) as exc:
                    return [
                        LogInfo(msg=str(exc)),
                        EmitEvent(
                            event=Shutdown(reason='localization database preparation failed')
                        ),
                    ]
            return [LogInfo(msg='Localization database ready; starting RTAB-Map'), rtab_action]

        if database_lease and database_lease.ready():
            actions.extend(
                [LogInfo(msg=f'Reusing private localization database: {database}'), rtab_action]
            )
        elif switches['copy_database']:
            actions.extend(
                [
                    RegisterEventHandler(
                        OnProcessExit(target_action=database_copy, on_exit=database_prepared)
                    ),
                    database_copy,
                ]
            )
        else:
            actions.extend(
                [
                    LogInfo(msg='Using source database directly; RTAB-Map may write to it'),
                    rtab_action,
                ]
            )
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch',
                            'navigation_launch.py',
                        )
                    ),
                    launch_arguments={
                        'params_file': params,
                        'autostart': 'true',
                        'use_composition': 'False',
                    }.items(),
                ),
                Node(
                    package='gps_navigator',
                    executable='localization_guard',
                    name='localization_guard',
                    output='screen',
                    parameters=[
                        params,
                        {
                            'goal_source': 'route' if needs_route else 'nav2',
                            # Normal navigation trusts native RTAB registration once,
                            # then tracks live odometry/TF. Research verdicts do not gate it.
                            'enable_coarse_hint_handoff': False,
                            'native_localization_only': not needs_route,
                            'stable_duration_s': (
                                0.5
                                if not needs_route
                                else nav_params.get('localization_guard', {})
                                .get('ros__parameters', {})
                                .get('stable_duration_s', 3.0)
                            ),
                            'occupancy_topic': '/navigation/route_map' if needs_route else '/map',
                            'footprint': json.dumps(footprint),
                            'footprint_padding': float(footprint_padding),
                        },
                    ],
                ),
            ]
        )
        if needs_route:
            actions.extend(
                [
                    Node(
                        package='gps_navigator',
                        executable='path_planner',
                        name='path_planner',
                        output='screen',
                        parameters=[
                            {
                                'navigation_profile': str(session / 'profile.resolved.yaml'),
                                'route_policy': policy,
                                'max_start_edge_distance_m': min(
                                    3.0, profile['route_half_width_m']
                                ),
                                'start_node_snap_distance_m': 0.5,
                                'start_edge_endpoint_margin_m': 0.5,
                                'destination': value('destination'),
                            }
                        ],
                    ),
                    Node(
                        package='gps_navigator',
                        executable='map_route_adapter',
                        name='map_route_adapter',
                        output='screen',
                        parameters=[
                            {
                                'navigation_profile': str(session / 'profile.resolved.yaml'),
                                'route_policy': policy,
                            }
                        ],
                    ),
                ]
            )
        if mode == 'drive':
            actions.append(
                Node(
                    package='gps_navigator',
                    executable='pixhawk_command_adapter',
                    name='pixhawk_command_adapter',
                    output='screen',
                    parameters=[
                        {
                            'cmd_vel_topic': '/cmd_vel_guarded',
                            'maximum_speed_m_s': 0.4,
                            'hold_on_fault': True,
                        }
                    ],
                )
            )
    if switches['rviz']:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', os.path.join(share, 'config', 'navigation.rviz')],
            )
        )
    actions.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': lifecycle}],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            *runtime_environment(),
            DeclareLaunchArgument('mode', default_value='planner'),
            # Shutdown timers evaluate launch configurations when shutdown starts,
            # after scoped groups have exited. Keep this configuration at root scope.
            # A large repaired dictionary needs more than the default five seconds.
            DeclareLaunchArgument('sigterm_timeout', default_value='120'),
            DeclareLaunchArgument('route_policy', default_value='direct'),
            DeclareLaunchArgument(
                'profile',
                default_value='',
                description='Legacy profile; defaults to maps/current.yaml',
            ),
            DeclareLaunchArgument(
                'map_dir',
                default_value='',
                description='Session with rtabmap.db + rtabmap.yaml/image',
            ),
            DeclareLaunchArgument('database', default_value=''),
            DeclareLaunchArgument('map', default_value=''),
            DeclareLaunchArgument('destination', default_value=''),
            DeclareLaunchArgument('start_mavros', default_value='false'),
            DeclareLaunchArgument('start_camera', default_value='false'),
            DeclareLaunchArgument(
                'start_camera_tf',
                default_value='true',
                description='Publish base_link -> camera_link; disable when provided externally',
            ),
            DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyACM0:115200'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'localization_file',
                default_value='',
                description='YAML camera/RTAB launch arguments',
            ),
            DeclareLaunchArgument(
                'record_camera',
                default_value='false',
                description='Record RGB-D inputs; also enables record_bag',
            ),
            DeclareLaunchArgument(
                'record_bag',
                default_value='false',
                description='Opt-in rosbag recording of navigation telemetry',
            ),
            DeclareLaunchArgument(
                'copy_database',
                default_value='true',
                description='Private DB copy; false lets RTAB write the source DB',
            ),
            DeclareLaunchArgument(
                'reuse_database',
                default_value='true',
                description='Reuse private DB; false creates a fresh session copy',
            ),
            DeclareLaunchArgument(
                'database_cache_root',
                default_value=os.path.expanduser('~/ros2_ws/.cache/localization'),
            ),
            DeclareLaunchArgument(
                'turn_speed_m_s',
                default_value='0.25',
                description='RPP turn floor [0.25, 0.35] m/s',
            ),
            DeclareLaunchArgument(
                'log_root', default_value=os.path.expanduser('~/ros2_ws/log/navigation')
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value='',
                description='Nav2 YAML; session nav2.yaml or package default',
            ),
            OpaqueFunction(function=setup_navigation),
        ]
    )
