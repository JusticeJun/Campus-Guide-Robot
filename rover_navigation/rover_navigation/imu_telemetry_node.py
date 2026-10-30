"""Request session-only attitude telemetry; never change stored FCU parameters."""

from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from sensor_msgs.msg import Imu


class ImuTelemetry(Node):
    def __init__(self):
        super().__init__('imu_telemetry')
        self.message_id = self.declare_parameter('message_id', 31).value
        topic = self.declare_parameter('imu_topic', '/mavros/imu/data').value
        self.connected = False
        self.samples = deque(maxlen=500)
        self.attempts = 0
        self.next_attempt = 0.0
        self.future = None
        self.client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.tick)

    def on_state(self, msg):
        if msg.connected != self.connected:
            self.connected = msg.connected
            self.samples.clear()
            self.attempts = 0
            self.next_attempt = time.monotonic() + 2.0
            if self.future is not None:
                self.client.remove_pending_request(self.future)
                self.future = None

    def on_imu(self, msg):
        self.samples.append(time.monotonic())

    def tick(self):
        now = time.monotonic()
        recent = [t for t in self.samples if now - t < 5.0]
        hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 else 0.0
        fresh = bool(recent) and now - recent[-1] < 0.5
        if not self.connected:
            return
        if self.future is not None:
            if self.future.done():
                try:
                    accepted = self.future.result().success
                except Exception as exc:
                    self.get_logger().warning(f'Attitude rate request failed: {exc}')
                else:
                    self.get_logger().info(
                        f'IMU message {self.message_id} 30 Hz request acknowledged: {accepted}; '
                        'measured rate is checked separately'
                    )
                self.future = None
            elif now >= self.next_attempt:
                self.client.remove_pending_request(self.future)
                self.future = None
            else:
                return
        # Always request once per connection. Recheck actual arrival, not only ACK.
        if self.attempts and hz >= 15.0 and fresh:
            self.get_logger().info(f'Pixhawk IMU: {hz:.1f} Hz', throttle_duration_sec=30.0)
            return
        if now >= self.next_attempt and self.attempts < 3 and self.client.service_is_ready():
            self.future = self.client.call_async(
                MessageInterval.Request(message_id=self.message_id, message_rate=30.0)
            )
            self.attempts += 1
            self.next_attempt = now + 15.0
        if self.attempts >= 3 or (self.attempts and hz < 15.0):
            self.get_logger().warning(
                f'Pixhawk IMU insufficient: {hz:.1f} Hz, fresh={fresh}; '
                'check telemetry before mapping',
                throttle_duration_sec=15.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
