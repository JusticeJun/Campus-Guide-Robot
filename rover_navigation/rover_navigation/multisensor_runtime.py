"""Shared sensor fusion and scan wiring for mapping and navigation."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from rover_navigation.rtab_runtime import odometry_node


def fused_nodes(profile, config):
    share = Path(get_package_share_directory('rover_navigation'))
    return [
        odometry_node(profile, config, fused=True),
        Node(package='rover_navigation', executable='imu_telemetry', name='raw_imu_telemetry',
             parameters=[{'message_id': 27, 'imu_topic': '/mavros/imu/data_raw'}],
             output='screen'),
        Node(package='rtabmap_odom', executable='icp_odometry', namespace='lio',
             name='icp_odometry', output='screen', parameters=[{
                 'frame_id': 'base_link', 'odom_frame_id': 'lidar_odom', 'publish_tf': False,
                 'wait_imu_to_init': False, 'qos': 2, 'topic_queue_size': 1,
                 'scan_range_min': 0.15, 'scan_range_max': 8.0,
                 'scan_voxel_size': 0.05, 'always_process_most_recent_frame': True,
                 'Reg/Force3DoF': 'true', 'Icp/PointToPlane': 'true',
                 'Icp/PointToPlaneK': '10', 'Icp/Iterations': '20',
                 'Icp/MaxCorrespondenceDistance': '0.3', 'Icp/CorrespondenceRatio': '0.2',
                 # Do not count a guess constrained by one wall as independent
                 # measured forward motion when camera tracking is unavailable.
                 'Icp/PointToPlaneLowComplexityStrategy': '0',
                 'Odom/ResetCountdown': '10', 'OdomF2M/ScanMaxSize': '2000',
             }], remappings=[('scan', '/scan_front')]),
        Node(package='robot_localization', executable='ekf_node', name='local_ekf',
             parameters=[str(share / 'config/local_ekf.yaml')], output='screen'),
        Node(package='rover_navigation', executable='fusion_inputs', name='fusion_inputs',
             output='screen'),
    ]


def scan_arguments(config):
    # Camera RGB/depth sync keeps its original 20ms bound in odometry_node.
    # RTAB's processed RGB-D + 5-10Hz scan need a separate, wider bound.
    return {
        'subscribe_scan': 'true', 'scan_topic': '/scan_front', 'qos_scan': '2',
        'approx_sync': 'true', 'approx_sync_max_interval': '0.2',
        'topic_queue_size': '30', 'sync_queue_size': '30',
        'odom_frame_id': 'odom', 'rgbd_topic': '/vo/odom_rgbd_image',
    }


def registration_arguments():
    # Visual registration seeds ICP refinement; keep visual geometric thresholds.
    # Scan data is archived for both localization and occupancy mapping.
    return (' --Reg/Strategy 2 --Reg/Force3DoF true'
            ' --Grid/Sensor 2 --Icp/PointToPlane true --Icp/Iterations 20'
            ' --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.3'
            ' --Icp/CorrespondenceRatio 0.2')
