"""RPLIDAR A1 front 200 degrees; independent of map/odometry/control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rover_navigation.rtab_runtime import runtime_environment


def generate_launch_description():
    return LaunchDescription([
        *runtime_environment(),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(package='rplidar_ros', executable='rplidar_node', name='rplidar',
             parameters=[{'serial_port': LaunchConfiguration('serial_port'),
                          'serial_baudrate': 115200, 'frame_id': 'laser',
                          'inverted': False, 'angle_compensate': True}],
             remappings=[('scan', '/scan_raw')], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', '0.32', '--y', '0', '--z', '0.18',
                        '--yaw', LaunchConfiguration('yaw'), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        Node(package='rover_navigation', executable='front_scan', output='screen',
             parameters=[{
                 'sensor_yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
             }]),
    ])
