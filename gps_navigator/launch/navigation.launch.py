import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('gps_navigator')
    nav2_share = get_package_share_directory('nav2_bringup')
    destination = LaunchConfiguration('destination')
    position_topic = LaunchConfiguration('position_topic')
    heading_topic = LaunchConfiguration('heading_topic')
    params_file = LaunchConfiguration('params_file')
    compact_console = LaunchConfiguration('compact_console')
    nav2_log_level = PythonExpression([
        "'warn' if '", compact_console, "'.lower() == 'true' else 'info'"
    ])
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
        DeclareLaunchArgument('compact_console', default_value='true'),
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
    ])
