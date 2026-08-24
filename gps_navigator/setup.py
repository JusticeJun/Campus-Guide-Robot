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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykk',
    maintainer_email='ykk@todo.todo',
    description='GPS waypoint graph planning and route progress management',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner = gps_navigator.path_planner_node:main',
            'route_manager = gps_navigator.route_manager_node:main',
            'waypoint_follower = gps_navigator.waypoint_follower_node:main',
        ],
    },
)
