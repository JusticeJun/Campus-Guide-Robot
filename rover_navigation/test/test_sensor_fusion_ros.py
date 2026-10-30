"""Real EKF + DDS with synthetic sensors; no FCU or actuator publisher."""

from pathlib import Path
import subprocess
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo

from rover_navigation.fusion_node import FusionInputs


def test_independent_loss_and_pose_reset_keep_continuous_measured_odometry(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '88')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces>'
                       '<AllowMulticast>false</AllowMulticast></General>'
                       '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<Peers><Peer Address="localhost"/></Peers></Discovery>'
                       '</Domain></CycloneDDS>')
    config = Path(__file__).parents[1] / 'config/local_ekf.yaml'
    log = (tmp_path / 'ekf.log').open('w')
    ekf = subprocess.Popen(['/opt/ros/humble/lib/robot_localization/ekf_node', '--ros-args',
                            '-r', '__node:=local_ekf', '--params-file', str(config)],
                           stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    fusion, source = FusionInputs(), Node('synthetic_motion')
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(source)
    pubs = {s: (source.create_publisher(Odometry, f'/{ns}/odom', 10),
                source.create_publisher(OdomInfo, f'/{ns}/odom_info_lite', 10))
            for s, ns in [('visual', 'vo'), ('lidar', 'lio')]}
    output, status = [], []
    source.create_subscription(Odometry, '/odom', output.append, qos_profile_sensor_data)
    source.create_subscription(OdomInfo, '/localization/odom_info', status.append,
                               qos_profile_sensor_data)
    start = time.monotonic()

    def run(duration, good, reset_camera=False, fixed_elapsed=None, lidar_bias=0.0, lidar_vy=0.0):
        end, next_publish = time.monotonic() + duration, 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                stamp = source.get_clock().now().to_msg()
                for name, (odom_pub, info_pub) in pubs.items():
                    info = OdomInfo()
                    info.header.stamp, info.lost = stamp, name not in good
                    info_pub.publish(info)
                    if name in good:
                        msg = Odometry()
                        msg.header.stamp, msg.header.frame_id = stamp, name + '_odom'
                        msg.child_frame_id = 'base_link'
                        msg.pose.pose.orientation.w = 1.0
                        msg.pose.pose.position.x = 0.3 * \
                            (now - start if fixed_elapsed is None else fixed_elapsed)
                        if name == 'lidar':
                            msg.pose.pose.position.x += lidar_bias
                        if name == 'visual' and reset_camera:
                            msg.pose.pose.position.x += 100.0
                        msg.twist.twist.linear.x = 0.3 if fixed_elapsed is None else 0.0
                        if name == 'lidar':
                            msg.twist.twist.linear.y = lidar_vy
                        for i in (0, 7, 14, 21, 28, 35):
                            msg.pose.covariance[i] = msg.twist.covariance[i] = 0.0025
                        odom_pub.publish(msg)
                next_publish = now + 0.1
            executor.spin_once(timeout_sec=0.01)

    try:
        run(4.0, {'visual', 'lidar'})
        assert len(output) > 20
        assert abs(output[-1].twist.twist.linear.x - 0.3) < 0.08
        status.clear()
        before = len(output)
        run(2.0, {'lidar'})
        assert len(output) > before + 20
        assert status and not any(m.lost for m in status)
        assert abs(output[-1].twist.twist.linear.x - 0.3) < 0.08
        before = len(output)
        run(2.0, {'visual', 'lidar'}, reset_camera=True)
        assert len(output) > before + 20
        xs = [m.pose.pose.position.x for m in output]
        assert max(abs(b - a) for a, b in zip(xs, xs[1:])) < 0.12
        assert output[-1].pose.covariance[0] < 0.1
        # Camera is unavailable and one short burst of invalid ICP velocity
        # arrives. Reject it without revoking the recent good scan immediately.
        status.clear()
        run(0.4, {'lidar'}, lidar_vy=.9)
        assert status and not any(m.lost for m in status)
        # Rejected scans must NOT renew the last-good timestamp indefinitely.
        run(1.8, {'lidar'}, lidar_vy=.9)
        assert status[-1].lost
        before = len(output)
        run(.3, {'lidar'}, lidar_vy=.9)
        assert len(output) == before
        run(1.0, {'lidar'})
        assert not status[-1].lost
        # Reproduce independent accumulated-position disagreement while both
        # sensors report a stopped rover. Lidar's +67 cm origin/drift must not
        # pull the EKF away from the camera pose or generate phantom speed.
        fixed_elapsed = time.monotonic()-start
        run(2.0, {'visual', 'lidar'}, reset_camera=True, fixed_elapsed=fixed_elapsed)
        stopped_x = output[-1].pose.pose.position.x
        run(3.0, {'visual', 'lidar'}, reset_camera=True,
            fixed_elapsed=fixed_elapsed, lidar_bias=.67)
        assert abs(output[-1].pose.pose.position.x-stopped_x) < .08
        assert max(abs(m.twist.twist.linear.x) for m in output[-20:]) < .05

        # Actual failure: ICP inferred near 0.9 m/s sideways while VO stayed still.
        before = len(output)
        run(2.0, {'visual', 'lidar'}, reset_camera=True,
            fixed_elapsed=fixed_elapsed, lidar_vy=.9)
        assert len(output) > before+15
        assert max(abs(m.twist.twist.linear.y) for m in output[-20:]) < .05
        assert abs(output[-1].pose.pose.position.y) < .08

        run(0.5, set())
        before = len(output)
        run(1.0, set())
        assert len(output) == before  # EKF predictions do not fake valid measurements.
        assert status[-1].lost
        assert ekf.poll() is None
    finally:
        executor.shutdown()
        fusion.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
        ekf.terminate()
        ekf.wait(timeout=10)
        log.close()
