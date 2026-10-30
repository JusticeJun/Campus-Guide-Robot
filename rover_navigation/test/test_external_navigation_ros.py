"""ROS loopback ExternalNav integration, isolated from the rover's DDS domain."""

import json
import os
import time

from geometry_msgs.msg import TwistStamped
from mavros.mavlink import convert_to_bytes
from mavros_msgs.msg import Mavlink, State
from mavros_msgs.srv import ParamPull
from nav_msgs.msg import Odometry
from pymavlink.dialects.v20 import ardupilotmega as dialect
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rover_navigation.external_navigation import FCU_PROFILE, ros_mavlink
from rover_navigation.external_navigation_node import ExternalNavigation
from test_external_navigation import odometry


def test_ardurover_status_uses_verified_fcu_packet_and_initialization_flag():
    """Wrong sender, corrupt packet and uninitialized EKF cannot establish readiness."""
    from types import SimpleNamespace

    received = []
    receiver = SimpleNamespace(decoder=dialect.MAVLink(None), on_estimator=received.append)

    def packet(flags, system=1):
        report = dialect.MAVLink_ekf_status_report_message(flags, 0.0, 0.0, 0.0, 0.0, 0.0)
        return ros_mavlink(report.pack(dialect.MAVLink(None, srcSystem=system, srcComponent=1)))

    ExternalNavigation.on_fcu_mavlink(receiver, packet(1 | 2 | 8, system=2))
    damaged = packet(1 | 2 | 8)
    damaged.checksum ^= 1
    ExternalNavigation.on_fcu_mavlink(receiver, damaged)
    assert not received
    ExternalNavigation.on_fcu_mavlink(receiver, packet(1 | 2 | 8 | 1024))
    assert not received[-1].attitude_status_flag
    ExternalNavigation.on_fcu_mavlink(receiver, packet(1 | 2 | 8))
    assert received[-1].attitude_status_flag
    assert received[-1].velocity_horiz_status_flag
    assert received[-1].pos_horiz_rel_status_flag


def test_wire_transport_parameter_audit_ready_and_tracking_loss(tmp_path, monkeypatch):
    """Exercise real DDS delivery and service callbacks without vehicle commands."""
    # Never use the live rover's domain or its MAVLink router sink.
    monkeypatch.setenv('ROS_DOMAIN_ID', '87')
    # Explicit lo selection below already restricts transport; do not add lo twice.
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo"/></Interfaces>'
        '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>',
    )
    assert os.environ['ROS_DOMAIN_ID'] == '87'
    rclpy.init(
        args=[
            '--ros-args',
            '-p',
            'transmit:=true',
            '-p',
            'mounting_reviewed:=true',
            '-p',
            'max_odometry_gap_s:=2.0',
            '-p',
            'mavlink_sink:=/externalnav_test_sink',
            '-p',
            f'reset_file:={tmp_path}/reset',
        ]
    )
    bridge = ExternalNavigation()
    mock = Node(
        'param',
        namespace='/mavros',
        use_global_arguments=False,
        parameter_overrides=[Parameter(k, value=float(v)) for k, v in FCU_PROFILE.items()],
        automatically_declare_parameters_from_overrides=True,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(bridge)
    packets, ready, statuses = [], [], []
    publishers = {
        topic: mock.create_publisher(kind, topic, qos_profile_sensor_data)
        for topic, kind in [
            ('/odom', Odometry),
            ('/rtabmap/odom_info_lite', OdomInfo),
            ('/mavros/state', State),
            ('/uas1/mavlink_source', Mavlink),
            ('/mavros/local_position/velocity_body', TwistStamped),
        ]
    }
    mock.create_subscription(
        Mavlink, '/externalnav_test_sink', packets.append, qos_profile_sensor_data
    )
    mock.create_subscription(Bool, '/external_navigation/ready', lambda m: ready.append(m.data), 1)
    mock.create_subscription(
        String, '/external_navigation/status', lambda m: statuses.append(json.loads(m.data)), 1
    )

    def pull(request, reply):
        reply.success, reply.param_received = True, len(FCU_PROFILE)
        return reply

    mock.create_service(ParamPull, '/mavros/param/pull', pull)
    lost, armed, pause_odom = [False], [False], [False]
    stale_odom = [False]
    ekf_bad = [False]
    odom_period, last_odom = [1 / 15], [0.0]

    def publish():
        stamp = mock.get_clock().now()
        msg = odometry(stamp.nanoseconds / 1e9 - (1.0 if stale_odom[0] else 0.0))
        info = OdomInfo(lost=lost[0])  # No arbitrary additional inlier-count gate.
        info.header.stamp = msg.header.stamp
        body = TwistStamped()
        body.header.stamp, body.header.frame_id = stamp.to_msg(), 'base_link'
        est = ros_mavlink(
            dialect.MAVLink_ekf_status_report_message(
                flags=(1 | 2 | 8) if not ekf_bad[0] else 1,
                velocity_variance=0.01,
                pos_horiz_variance=0.01,
                pos_vert_variance=0.01,
                compass_variance=0.0,
                terrain_alt_variance=0.0,
            ).pack(dialect.MAVLink(None, srcSystem=1, srcComponent=1))
        )
        publishers['/mavros/state'].publish(
            State(connected=True, armed=armed[0], mode='GUIDED' if armed[0] else 'HOLD')
        )
        publishers['/uas1/mavlink_source'].publish(est)
        publishers['/mavros/local_position/velocity_body'].publish(body)
        if not pause_odom[0] and time.monotonic() - last_odom[0] >= odom_period[0]:
            publishers['/odom'].publish(msg)
            publishers['/rtabmap/odom_info_lite'].publish(info)
            last_odom[0] = time.monotonic()

    timer = mock.create_timer(1 / 15, publish)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    try:
        spin(5.5)
        assert packets and any(ready), statuses[-3:]
        decoded = [dialect.MAVLink(None).parse_char(bytes(convert_to_bytes(m))) for m in packets]
        assert all(m.frame_id == 20 and m.child_frame_id == 12 for m in decoded)
        stamps = [m.time_usec for m in decoded]
        assert stamps == sorted(set(stamps))
        assert all(m.reset_counter == 1 and m.quality == 1 for m in decoded)
        stale_odom[0] = True
        count = len(packets)
        spin(0.25)
        assert len(packets) <= count + 1  # At most one already queued valid packet.
        assert ready[-1] and not bridge.fault and bridge.stale_samples_dropped
        stale_odom[0] = False
        spin(0.3)
        assert ready[-1] and len(packets) > count and not bridge.fault
        armed[0] = True
        spin(0.3)
        pause_odom[0] = True
        ready_before = len(ready)
        spin(1.7)
        assert all(ready[ready_before:]) and not bridge.fault
        count = len(packets)
        pause_odom[0] = False
        spin(0.4)
        assert ready[-1] and len(packets) > count and not bridge.fault
        odom_period[0] = 0.35  # About 2.5 Hz through the 15 Hz timer.
        armed[0] = True
        count = len(packets)
        ready_before = len(ready)
        spin(4.0)
        assert len(packets) > count and not bridge.fault
        # A rate dip below the startup threshold does not revoke an already
        # established healthy session, even while armed. No HOLD-triggering false.
        assert all(ready[ready_before:])
        assert ready[-1] and statuses[-1]['fcu_ready']
        assert not statuses[-1]['input_ready'] and statuses[-1]['continuing_healthy']
        ekf_bad[0] = True
        spin(0.3)
        assert not ready[-1] and not bridge.operational
        ekf_bad[0] = False
        spin(2.5)
        assert not ready[-1]  # Real FCU failure requires startup admission again.
        bridge.set_parameters([Parameter('minimum_ready_rate_hz', value=2.0)])
        spin(2.5)
        assert ready[-1] and not bridge.fault
        assert 2.0 <= statuses[-1]['input_hz'] < 4.0
        armed[0] = False
        odom_period[0] = 1 / 15
        spin(2.5)
        pause_odom[0] = True
        spin(2.1)
        # The timer can detect the fault on the final spin; allow its queued
        # DDS false heartbeat to reach the subscriber before checking it.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and (not bridge.fault or ready[-1]):
            executor.spin_once(timeout_sec=0.02)
        assert bridge.fault and bridge.auto_recoverable and not ready[-1]
        count = len(packets)
        pause_odom[0] = False
        spin(0.4)
        assert len(packets) > count and not ready[-1]
        spin(4.6)
        assert not bridge.fault and ready[-1]  # Disarmed startup resumes without a service.
        armed[0] = True
        spin(0.2)
        lost[0] = True
        count = len(packets)
        ready_before = len(ready)
        spin(1.0)
        assert all(ready[ready_before:]) and not bridge.fault
        assert len(packets) <= count + 1  # Invalid input never becomes FCU data.
        lost[0] = False
        spin(0.4)
        assert ready[-1] and not bridge.fault and len(packets) > count
        # Repeated lost messages cannot refresh the valid-sample watchdog.
        lost[0] = True
        spin(2.3)
        assert ready[-1] is False and bridge.fault
        assert not bridge.auto_recoverable
        count = len(packets)
        armed[0] = False
        lost[0] = False
        spin(0.5)
        assert len(packets) == count and not ready[-1]  # No automatic restart.
        client = mock.create_client(Trigger, '/external_navigation/recover')
        pending = client.call_async(Trigger.Request())
        spin(0.2)
        assert pending.done() and not pending.result().success  # Too early.
        spin(2.0)
        pending = client.call_async(Trigger.Request())
        spin(0.2)
        assert pending.done() and pending.result().success
        spin(2.5)
        assert len(packets) > count and ready[-1] and not bridge.fault
        assert bridge.reset_counter == 1  # No coordinate reset was introduced.
        bridge.fail('odometry position discontinuity')
        spin(2.5)
        old_count = len(packets)
        reset_client = mock.create_client(Trigger, '/external_navigation/reinitialize')
        pending = reset_client.call_async(Trigger.Request())
        spin(.2)
        assert pending.done() and pending.result().success
        assert bridge.reset_counter == 2 and (tmp_path/'reset').read_text() == '2'
        # New epoch requires the sensor window and then FCU settling.
        spin(4.5)
        assert len(packets) > old_count and ready[-1] and not bridge.fault

    finally:
        mock.destroy_timer(timer)
        executor.remove_node(bridge)
        executor.remove_node(mock)
        bridge.destroy_node()
        mock.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
