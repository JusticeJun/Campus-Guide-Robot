import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('gps_navigator')
    nav2_share = get_package_share_directory('nav2_bringup')
    destination = LaunchConfiguration('destination')
    position_topic = LaunchConfiguration('position_topic')
    heading_topic = LaunchConfiguration('heading_topic')
    params_file = LaunchConfiguration('params_file')
    record_bag = LaunchConfiguration('record_bag')
    compact_console = LaunchConfiguration('compact_console')
    dashboard_in_launch = LaunchConfiguration('dashboard_in_launch')
    nav2_log_level = PythonExpression([
        "'warn' if '", compact_console, "'.lower() == 'true' else 'info'"
    ])
    bag_path = os.path.join(
        '/home/ykk/ros2_ws/diagnostics',
        'nav2_production_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
    )
    ackermann_through_poses_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_through_poses_ackermann.xml',
    )
    ackermann_to_pose_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_to_pose_ackermann.xml',
    )
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_through_poses_bt_xml': ackermann_through_poses_bt,
            'default_nav_to_pose_bt_xml': ackermann_to_pose_bt,
        },
        convert_types=True,
    )

    origin_parameters = {
        'origin_latitude': 35.13484300,
        'origin_longitude': 129.1038000,
    }
    localization_parameters = {
        'position_topic': position_topic,
        'heading_topic': heading_topic,
        **origin_parameters,
    }
    return LaunchDescription([
        DeclareLaunchArgument('destination', default_value=''),
        DeclareLaunchArgument(
            'position_topic',
            default_value='/mavros/global_position/global',
        ),
        DeclareLaunchArgument(
            'heading_topic',
            default_value='/mavros/global_position/compass_hdg',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_gps.yaml'
            ),
        ),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('compact_console', default_value='true'),
        DeclareLaunchArgument('dashboard_in_launch', default_value='true'),
        Node(
            package='gps_navigator', executable='gps_localization',
            name='gps_localization', output='screen',
            parameters=[localization_parameters],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': configured_params,
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'False',
                'log_level': nav2_log_level,
            }.items(),
        ),
        Node(
            package='gps_navigator', executable='path_planner',
            name='path_planner', output='screen',
            parameters=[{
                'destination': destination,
                'position_topic': position_topic,
            }],
        ),
        Node(
            package='gps_navigator', executable='nav2_route_adapter',
            name='nav2_route_adapter', output='screen',
            parameters=[origin_parameters],
        ),
        Node(
            package='gps_navigator', executable='pixhawk_command_adapter',
            name='pixhawk_command_adapter', output='screen',
        ),
        Node(
            condition=IfCondition(dashboard_in_launch),
            package='gps_navigator', executable='navigation_status_monitor',
            name='navigation_status_monitor', output='screen',
            emulate_tty=True,
            parameters=[{
                'position_topic': position_topic,
                'heading_topic': heading_topic,
                'status_rate_hz': 1.0,
                'dashboard_mode': ParameterValue(
                    compact_console, value_type=bool
                ),
            }],
        ),
        ExecuteProcess(
            condition=IfCondition(record_bag),
            output='screen',
            cmd=[
                'ros2', 'bag', 'record', '--include-hidden-topics',
                '-o', bag_path,
                '/gps_navigation/route',
                '/navigate_through_poses/_action/feedback',
                '/plan',
                '/local_plan',
                '/cmd_vel_nav',
                '/cmd_vel',
                '/mavros/setpoint_raw/local',
                '/mavros/setpoint_raw/target_local',
                '/mavros/local_position/velocity_body',
                '/mavros/imu/data',
                '/mavros/global_position/global',
                '/mavros/global_position/compass_hdg',
                '/mavros/rc/out',
                '/mavros/mavlink/from',
                '/mavros/state',
                '/odometry/gps',
                '/tf',
                '/tf_static',
            ],
        ),
    ])
