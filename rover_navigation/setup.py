import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
            + glob('config/*.rviz')
            + glob('config/*.xml')
            + glob('config/*.sh')
            + glob('config/*.param'),
        ),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykk',
    maintainer_email='ykk@todo.todo',
    description='Mapping, localization and Nav2 integration for the campus Ackermann rover',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'static_clearance = rover_navigation.static_clearance_node:main',
            'fusion_inputs = rover_navigation.fusion_node:main',
            'front_scan = rover_navigation.lidar_filter_node:main',
            'imu_telemetry = rover_navigation.imu_telemetry_node:main',
            'external_navigation = rover_navigation.external_navigation_node:main',
            'path_planner = rover_navigation.path_planner_node:main',
            'localization_guard = rover_navigation.localization_guard_node:main',
            'drive_trace = rover_navigation.drive_trace_node:main',
            'map_route_adapter = rover_navigation.map_route_adapter_node:main',
            'navigation_profile = rover_navigation.navigation_profile:main',
            'turn_radius_calibration = ' 'rover_navigation.turn_radius_calibration_node:main',
            'speed_calibration = ' 'rover_navigation.speed_calibration_node:main',
            'steering_pid_recorder = ' 'rover_navigation.steering_pid_recorder_node:main',
            'gps_localization = rover_navigation.gps_localization_node:main',
            'nav2_route_adapter = rover_navigation.nav2_route_adapter_node:main',
            'pixhawk_command_adapter = ' 'rover_navigation.pixhawk_command_adapter_node:main',
            'navigation_status_monitor = ' 'rover_navigation.navigation_status_monitor_node:main',
            'gps_waypoint_sampler = ' 'rover_navigation.gps_waypoint_sampler_node:main',
        ],
    },
)
