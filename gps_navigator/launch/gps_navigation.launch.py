from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    destination = LaunchConfiguration('destination')
    position_topic = LaunchConfiguration('position_topic')
    arrival_radius = LaunchConfiguration('arrival_radius_m')
    use_follower = LaunchConfiguration('use_waypoint_follower')

    return LaunchDescription([
        DeclareLaunchArgument('destination', default_value=''),
        DeclareLaunchArgument(
            'position_topic',
            default_value='/mavros/global_position/global',
        ),
        DeclareLaunchArgument('arrival_radius_m', default_value='3.0'),
        DeclareLaunchArgument(
            'use_waypoint_follower',
            default_value='false',
            description='Enable temporary MAVROS Rover control node',
        ),
        Node(
            package='gps_navigator',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'destination': destination,
                'position_topic': position_topic,
            }],
        ),
        Node(
            package='gps_navigator',
            executable='route_manager',
            name='route_manager',
            output='screen',
            parameters=[{
                'position_topic': position_topic,
                'arrival_radius_m': ParameterValue(
                    arrival_radius, value_type=float
                ),
            }],
        ),
        Node(
            package='gps_navigator',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            condition=IfCondition(use_follower),
        ),
    ])
