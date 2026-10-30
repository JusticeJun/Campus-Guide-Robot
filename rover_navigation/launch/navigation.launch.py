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

from rover_navigation.rtab_runtime import odometry_node, runtime_environment
from rover_navigation.map_session import read_mapping, select_session
from rover_navigation.localization_database import LocalizationDatabase, full_working_memory_saved
from rover_navigation.path_validation import apply_obstacle_clearance, shared_footprint
from rover_navigation.speed_policy import apply_speed_policy
from rover_navigation.startup_pose import load_startup_pose
from rover_navigation.multisensor_runtime import (
    fused_nodes, scan_arguments, registration_arguments,
)
from rover_navigation.lidar_navigation import configure_obstacles


def allow_long_planner_search(params):
    """Allow distant-goal searches, including maps saved with older limits."""
    planner = params.get('planner_server', {}).get('ros__parameters', {})
    for name in planner.get('planner_plugins', []):
        plugin = planner.get(name, {})
        if plugin.get('plugin') == 'nav2_smac_planner/SmacPlannerHybrid':
            plugin['max_iterations'] = -1
            # Humble interprets zero seconds as an immediate timeout.
            plugin['max_planning_time'] = 1000000000.0


def apply_turning_geometry(params):
    """Plan at 2 m, keeping 1.7 m command curvature available for correction."""
    planner = params.get('planner_server', {}).get('ros__parameters', {})
    for name in planner.get('planner_plugins', []):
        plugin = planner.get(name, {})
        if plugin.get('plugin') == 'nav2_smac_planner/SmacPlannerHybrid':
            plugin['minimum_turning_radius'] = 2.0
            plugin['analytic_expansion_max_length'] = max(
                10.0, float(plugin.get('analytic_expansion_max_length', 10.0))
            )


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
    fcu_estimation = value('fcu_estimation')
    if fcu_estimation not in ('gps', 'externalnav'):
        raise ValueError('fcu_estimation must be gps or externalnav')
    externalnav = fcu_estimation == 'externalnav' and mode != 'planner'
    transmit = value('externalnav_transmit').lower()
    if transmit == 'auto':
        transmit = 'true' if mode in ('drive', 'validation') else 'false'
    if transmit not in ('true', 'false'):
        raise ValueError('externalnav_transmit must be auto, true or false')
    if externalnav and mode == 'drive' and transmit != 'true':
        raise ValueError('ExternalNav drive requires externalnav_transmit:=true')
    switches = {}
    for name in (
        'start_mavros',
        'start_camera',
        'start_lidar',
        'start_camera_tf',
        'rviz',
        'record_bag',
        'copy_database',
        'reuse_database',
        'use_startup_pose',
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
    scan_map = runtime.get('sensor_mode', 'rgbd') == 'rgbd_lidar'
    odom_choice = value('local_odometry')
    if odom_choice not in ('auto', 'camera', 'fused'):
        raise ValueError('local_odometry must be auto, camera or fused')
    fused = scan_map if odom_choice == 'auto' else odom_choice == 'fused'
    obstacle_choice = value('lidar_obstacles')
    if obstacle_choice not in ('auto', 'true', 'false'):
        raise ValueError('lidar_obstacles must be auto, true or false')
    lidar_obstacles = (
        (scan_map or fused) if obstacle_choice == 'auto' else obstacle_choice == 'true'
    )
    if scan_map and not fused:
        raise ValueError('Camera+lidar maps require their fused local odometry profile')
    health_remaps = ([('/rtabmap/odom_info_lite', '/localization/odom_info')]
                     if fused else [])
    memory_mode = value('localization_memory')
    if memory_mode not in ('auto', 'full'):
        raise ValueError('localization_memory must be auto or full')
    if mode != 'planner' and memory_mode == 'auto' and not switches['copy_database']:
        raise ValueError('automatic memory management requires a private database copy')
    startup_pose, startup_message = '', ''
    if mode != 'planner' and switches['use_startup_pose']:
        startup_pose, startup_message = load_startup_pose(
            Path(profile_path).parent / 'startup_pose.yaml', profile['database'], profile['map']
        )
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
    allow_long_planner_search(nav_params)
    apply_turning_geometry(nav_params)
    if lidar_obstacles:
        configure_obstacles(nav_params)
    apply_obstacle_clearance(nav_params, lidar_obstacles=lidar_obstacles)
    # Selected only by campus observation missions; existing RViz/default
    # FollowPath goal checker and its tolerances are unchanged.
    goal_params = nav_params['controller_server']['ros__parameters']
    goal_plugins = goal_params.setdefault('goal_checker_plugins', ['goal_checker'])
    if 'observation_goal_checker' not in goal_plugins:
        goal_plugins.append('observation_goal_checker')
    goal_params['observation_goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker',
        'stateful': True, 'xy_goal_tolerance': 0.45, 'yaw_goal_tolerance': 0.25,
    }
    cache_static_planner_heuristic(nav_params, needs_route)
    # Apply to old per-map YAML too. Validation exercises the same guard as drive.
    nav_params.setdefault('localization_guard', {}).setdefault('ros__parameters', {}).update(
        {'strict_tracking_safety': True, 'tracking_max_age_s': 2.5,
         'covariance_grace_s': 1.0 if fused else 0.0,
         'tracking_loss_grace_s': 1.0 if fused else 0.0}
    )
    if externalnav:
        # Match the bounded ExternalNav input-gap budget. The bridge still
        # enforces its configured cutoff and FCU health independently.
        nav_params['localization_guard']['ros__parameters']['odometry_timeout_s'] = 2.0
    footprint, footprint_padding = shared_footprint(nav_params)
    share = get_package_share_directory('rover_navigation')
    session = Path(value('log_root')).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    session.mkdir(parents=True)
    (session / 'profile.resolved.yaml').write_text(yaml.safe_dump(profile))
    (session / 'run.yaml').write_text(
        yaml.safe_dump(
            {
                'mode': mode,
                'fcu_estimation': fcu_estimation,
                'externalnav_transmit': externalnav and transmit == 'true',
                'route_policy': policy,
                'destination': value('destination'),
                'goal_input': 'legacy_route' if needs_route else 'nav2_action',
                'startup_pose': startup_pose,
                'startup_pose_message': startup_message,
                'localization_memory': memory_mode,
                'local_odometry': 'fused' if fused else 'camera',
                'lidar_obstacles': lidar_obstacles,
                'map_sensor_mode': runtime.get('sensor_mode', 'rgbd'),
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
    if lidar_obstacles and not needs_route:
        rewrites['default_nav_to_pose_bt_xml'] = os.path.join(
            share, 'behavior_trees', 'navigate_lidar.xml')
    for name in ('local_costmap', 'global_costmap'):
        rewrites[f'{name}.{name}.ros__parameters.route_layer.enabled'] = (
            'true' if mode != 'planner' and needs_route else 'false'
        )
    params = RewrittenYaml(
        source_file=params_file, root_key='', convert_types=True, param_rewrites=rewrites
    )
    actions = [LogInfo(msg=f'Navigation session: {session}')]
    if mode != 'planner':
        if externalnav:
            external_config = value('externalnav_config') or os.path.join(
                share, 'config', 'external_navigation.yaml'
            )
            actions.append(
                Node(
                    package='rover_navigation',
                    executable='external_navigation',
                    name='external_navigation',
                    output='screen',
                    parameters=[external_config, {'transmit': transmit == 'true'}],
                    remappings=health_remaps,
                )
            )
        actions.append(
            Node(
                package='rover_navigation',
                executable='drive_trace',
                name='drive_trace',
                remappings=([('/rtabmap/odom_info_lite', '/vo/odom_info_lite')]
                            if fused else []),
                prefix='nice -n 10',
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
                            # navigation owns this TF via start_camera_tf.
                            launch_arguments={
                                'runtime_file': localization_file,
                                'publish_camera_tf': 'false',
                            }.items(),
                        ),
                    ],
                    forwarding=False,
                )
            )
        if switches['start_lidar']:
            actions.append(GroupAction([
                *runtime_environment(),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        share, 'launch', 'lidar.launch.py')),
                    launch_arguments={
                        'serial_port': value('lidar_port'), 'yaw': value('lidar_yaw'),
                    }.items(),
                ),
            ], forwarding=False))
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
    if lidar_obstacles:
        actions.append(Node(
            package='rover_navigation', executable='static_clearance', name='static_clearance',
            output='screen', parameters=[{'extra_margin_m': 0.0}],
        ))
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
        actions.extend(fused_nodes(profile, rtab_config) if fused
                       else [odometry_node(profile, rtab_config)])
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
        # VO must remain responsive regardless of the selected FCU speed source.
        # Map matching can wait for CPU without changing geometric checks.
        # The odometry node above stays at normal priority.
        rtab_args['launch_prefix'] = 'nice -n 5'
        if fused:
            rtab_args.update({'rgbd_topic': '/vo/odom_rgbd_image', 'odom_frame_id': 'odom'})
        if scan_map:
            rtab_args.update(scan_arguments(rtab_config))
        if startup_pose:
            # One startup guess, not a TF publisher or an accepted-localization event.
            # RViz /initialpose can still replace it; no localization is fabricated.
            rtab_args['initial_pose'] = startup_pose
        elif not switches['use_startup_pose']:
            rtab_args['initial_pose'] = ''
        if startup_message:
            actions.append(LogInfo(msg=startup_message))
        local_memory = memory_mode == 'auto' and bool(startup_pose)
        # Reuse RTAB's own index, without narrowing the set of searchable nodes.
        # A first/unfinished session still loads all nodes; the next clean full
        # checkpoint can restore the same coverage and its serialized index.
        # Local seeding replaces the checkpoint/index anyway; avoid scanning
        # every Word row in a large map just to decide whether to reuse it.
        rtab_args['args'] = rtab_config.get('args', '') + ' --Kp/FlannIndexSaved true'
        if scan_map:
            rtab_args['args'] += registration_arguments()
        # A seeded subset is enabled only after the asynchronous preparation
        # below succeeds. Keep VO extraction and geometric validation unchanged.
        memory_policy = read_mapping(Path(share) / 'config/localization_memory.yaml')
        memory_parameters = memory_policy['parameters']
        if not 30 <= int(memory_policy['startup_max_nodes']) <= int(
            memory_parameters['Rtabmap/MemoryThr']
        ):
            raise ValueError('startup memory must contain 30..MemoryThr nodes')
        if memory_mode == 'auto':
            rtab_args['args'] += (
                ' --Mem/LoadVisualLocalFeaturesOnInit false --Rtabmap/PublishRAMUsage true'
            )
        else:
            rtab_args['args'] += ''.join(
                f' --{key} {str(setting).lower()}'
                for key, setting in memory_policy['full_parameters'].items()
            )
        if local_memory:
            rtab_args['args'] += ''.join(
                f' --{key} {str(setting).lower()}' for key, setting in memory_parameters.items()
            )
        else:
            rtab_args['args'] += ' --Rtabmap/MemoryThr 0 --Rtabmap/TimeThr 0'

        memory_report = session / 'memory.json'
        memory_prepare = ExecuteProcess(
            cmd=['python3', '-m', 'rover_navigation.localization_memory',
                 '--database', str(database), '--pose', startup_pose,
                 '--source', profile['database'], '--mode', memory_mode,
                 '--radius', str(memory_policy['startup_radius_m']),
                 '--nodes', str(memory_policy['startup_max_nodes']),
                 '--report', str(memory_report)],
            name='prepare_localization_memory', output='screen',
        )

        def memory_prepared(event, context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='localization memory preparation failed'))]
            report = json.loads(memory_report.read_text())
            # IncludeLaunchDescription captured its launch arguments at creation.
            # Build it after preparation so native startup sees the chosen set.
            if report['seeded']:
                # This index is invalidated by the next startup selection.
                # Native serialization allocates a temporary 1 GiB buffer;
                # do not spend RAM/I/O saving an index we cannot reuse.
                rtab_args['args'] += (
                    ' --Mem/InitWMWithAllNodes false --Optimizer/PriorsIgnored false'
                    ' --Kp/FlannIndexSaved false'
                )
            else:
                # Cold/interrupted source must finish a complete checkpoint
                # before native checkpoint/index reuse. Decide AFTER preparation:
                # restoring an experimental subset can change its membership.
                # Never override a valid full checkpoint with a forced rebuild.
                restore_memory = full_working_memory_saved(database)
                rtab_args['args'] += (
                    ' --Mem/InitWMWithAllNodes ' + str(not restore_memory).lower()
                    + ' --Rtabmap/MemoryThr 0'
                )
            return [LogInfo(msg=f'Localization memory: {report}'), make_rtab_action()]

        def make_rtab_action():
            return GroupAction(
                [
                    *runtime_environment(),
                    SetParameter(name='pub_loc_pose_only_when_localizing', value=True),
                    SetParameter(name='map_cache_loaded_on_init', value=False),
                    SetRemap(src='initialpose', dst='/initialpose'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(
                            get_package_share_directory('rtabmap_launch'),
                            'launch', 'rtabmap.launch.py')),
                        launch_arguments=rtab_args.items(),
                    ),
                ], forwarding=False,
            )

        def start_localization():
            if local_memory or database.with_suffix('.global_graph.json').exists():
                return [RegisterEventHandler(OnProcessExit(
                    target_action=memory_prepare, on_exit=memory_prepared)), memory_prepare]
            # No adapter sidecar: still select the native saved full index after
            # the asynchronous copy, rather than capturing arguments beforehand.
            restore_memory = full_working_memory_saved(database)
            rtab_args['args'] += (
                ' --Mem/InitWMWithAllNodes ' + str(not restore_memory).lower()
            )
            return [make_rtab_action()]

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
            return [LogInfo(msg='Localization database ready'), *start_localization()]

        if database_lease and database_lease.ready():
            actions.extend(
                [LogInfo(msg=f'Reusing private localization database: {database}'),
                 *start_localization()]
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
                    *start_localization(),
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
                    package='rover_navigation',
                    executable='localization_guard',
                    name='localization_guard',
                    output='screen',
                    remappings=health_remaps,
                    parameters=[
                        params,
                        {
                            'goal_source': 'route' if needs_route else 'nav2',
                            'require_externalnav': externalnav,
                            'maximum_reverse_speed_m_s': 0.4 if lidar_obstacles else 0.0,
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
                        package='rover_navigation',
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
                        package='rover_navigation',
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
        if lidar_obstacles:
            actions.extend([
                Node(package='nav2_collision_monitor', executable='collision_monitor',
                     name='collision_monitor', output='screen',
                     parameters=[os.path.join(share, 'config/collision_monitor.yaml')]),
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_collision', output='screen',
                     parameters=[{'autostart': True, 'node_names': ['collision_monitor']}]),
            ])
        if mode == 'drive':
            actions.append(
                Node(
                    package='rover_navigation',
                    executable='pixhawk_command_adapter',
                    name='pixhawk_command_adapter',
                    output='screen',
                    parameters=[
                        {
                            'cmd_vel_topic': ('/cmd_vel_vehicle' if lidar_obstacles
                                              else '/cmd_vel_guarded'),
                            'maximum_speed_m_s': 0.4,
                            'maximum_reverse_speed_m_s': 0.4 if lidar_obstacles else 0.0,
                            'minimum_turning_radius_m': 1.7,
                            'hold_on_fault': True,
                            'require_externalnav': externalnav,
                            'require_scan': lidar_obstacles,
                        }
                    ],
                )
            )
    if switches['rviz']:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                prefix='nice -n 10',
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
            DeclareLaunchArgument('fcu_estimation', default_value='externalnav'),
            DeclareLaunchArgument('externalnav_transmit', default_value='auto'),
            DeclareLaunchArgument('externalnav_config', default_value=''),
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
            DeclareLaunchArgument('start_mavros', default_value='false',
                                  description='Opt in only when MAVROS is not running separately'),
            DeclareLaunchArgument('start_camera', default_value='false',
                                  description=(
                                      'Opt in only when camera.launch.py '
                                      'is not running separately')),
            DeclareLaunchArgument('start_lidar', default_value='false',
                                  description=(
                                      'Opt in only when lidar.launch.py '
                                      'is not running separately')),
            DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
            DeclareLaunchArgument(
                'start_camera_tf',
                default_value=LaunchConfiguration('start_camera'),
                description='Defaults to start_camera: external camera owns its TF; '
                            'embedded camera gets exactly one mounting TF here',
            ),
            DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyACM0:115200'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'local_odometry', default_value='auto',
                description='auto follows the map sensor profile; camera or fused'),
            DeclareLaunchArgument('lidar_obstacles', default_value='auto',
                                  description='auto enables scan avoidance with fused odometry'),
            DeclareLaunchArgument(
                'use_startup_pose', default_value='true',
                description='Use this map\'s verified startup_pose.yaml as a one-time RTAB hint',
            ),
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
                'localization_memory', default_value='full',
                description='full: production full-map loading; auto: experimental bounded WM',
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
