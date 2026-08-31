import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gps_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykk',
    maintainer_email='ykk@todo.todo',
    description='GPS route planning and Nav2 integration for an Ackermann rover',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner = gps_navigator.path_planner_node:main',
            'turn_radius_calibration = '
            'gps_navigator.turn_radius_calibration_node:main',
            'speed_calibration = '
            'gps_navigator.speed_calibration_node:main',
            'steering_pid_recorder = '
            'gps_navigator.steering_pid_recorder_node:main',
            'gps_localization = gps_navigator.gps_localization_node:main',
            'nav2_route_adapter = gps_navigator.nav2_route_adapter_node:main',
            'pixhawk_command_adapter = '
            'gps_navigator.pixhawk_command_adapter_node:main',
            'navigation_status_monitor = '
            'gps_navigator.navigation_status_monitor_node:main',
            'gps_waypoint_sampler = '
            'gps_navigator.gps_waypoint_sampler_node:main',
        ],
    },
)
