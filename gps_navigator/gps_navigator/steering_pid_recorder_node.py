import csv
from pathlib import Path
import time

from mavros_msgs.msg import Mavlink
from mavros_msgs.srv import EndpointAdd, EndpointDel
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from gps_navigator.turn_radius_calibration_node import (
    decode_pid_tuning,
    message_stamp_ns,
)


STEERING_PID_AXIS = 5
CSV_FIELDS = [
    'monotonic_time_ns',
    'ros_time_ns',
    'message_time_ns',
    'desired_deg_s',
    'achieved_deg_s',
    'ff',
    'p',
    'i',
    'd',
]


def steering_pid_tuning(msg):
    tuning = decode_pid_tuning(msg)
    if tuning is None or tuning['axis'] != STEERING_PID_AXIS:
        return None
    return tuning


class SteeringPidRecorder(Node):

    def __init__(self):
        super().__init__('steering_pid_recorder')
        self.declare_parameter('diagnostic_csv', '')
        self.declare_parameter(
            'mavros_router_add_endpoint_service',
            '/mavros/mavros_router/add_endpoint',
        )
        self.declare_parameter(
            'mavros_router_del_endpoint_service',
            '/mavros/mavros_router/del_endpoint',
        )

        self.endpoint_id = None
        self.endpoint_pending = False
        self.endpoint_prefix = (
            f'/steering_pid_recorder_mavlink_{time.monotonic_ns()}'
        )
        self.output = None
        self.writer = None
        self.csv_path = self.open_csv()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Mavlink,
            f'{self.endpoint_prefix}/mavlink_source',
            self.mavlink_callback,
            qos,
        )
        self.endpoint_add_client = self.create_client(
            EndpointAdd,
            self.get_parameter(
                'mavros_router_add_endpoint_service'
            ).value,
        )
        self.endpoint_del_client = self.create_client(
            EndpointDel,
            self.get_parameter(
                'mavros_router_del_endpoint_service'
            ).value,
        )
        self.create_timer(0.5, self.ensure_endpoint)
        self.get_logger().info(
            f'Steering PID_TUNING recorder ready: CSV={self.csv_path}'
        )

    def open_csv(self):
        configured = str(self.get_parameter('diagnostic_csv').value).strip()
        path = Path(configured) if configured else Path(
            '/tmp/nav2_steering_pid_%s.csv'
            % time.strftime('%Y%m%d_%H%M%S')
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        self.output = path.open(
            'w', newline='', encoding='utf-8', buffering=1
        )
        self.writer = csv.DictWriter(self.output, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        return path

    def ensure_endpoint(self):
        if self.endpoint_id is not None or self.endpoint_pending:
            return
        if not self.endpoint_add_client.service_is_ready():
            return
        request = EndpointAdd.Request()
        request.url = self.endpoint_prefix
        request.type = EndpointAdd.Request.TYPE_UAS
        future = self.endpoint_add_client.call_async(request)
        self.endpoint_pending = True
        future.add_done_callback(self.endpoint_callback)

    def endpoint_callback(self, future):
        self.endpoint_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Could not add MAVLink diagnostic endpoint: {error}'
            )
            return
        if not response.successful:
            self.get_logger().error(
                'Could not add MAVLink diagnostic endpoint: '
                f'{response.reason}'
            )
            return
        self.endpoint_id = response.id
        self.get_logger().info(
            'MAVLink PID_TUNING capture connected through router endpoint '
            f'{self.endpoint_prefix} (id={response.id})'
        )

    def mavlink_callback(self, msg):
        tuning = steering_pid_tuning(msg)
        if tuning is None:
            return
        self.writer.writerow({
            'monotonic_time_ns': time.monotonic_ns(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'message_time_ns': message_stamp_ns(msg),
            'desired_deg_s': tuning['desired'],
            'achieved_deg_s': tuning['achieved'],
            'ff': tuning['ff'],
            'p': tuning['p'],
            'i': tuning['i'],
            'd': tuning['d'],
        })

    def remove_endpoint(self):
        if (
            self.endpoint_id is None
            or not self.endpoint_del_client.service_is_ready()
        ):
            return None
        request = EndpointDel.Request()
        request.id = self.endpoint_id
        request.type = EndpointDel.Request.TYPE_UAS
        future = self.endpoint_del_client.call_async(request)
        self.endpoint_id = None
        return future

    def destroy_node(self):
        if self.output is not None:
            self.output.flush()
            self.output.close()
            self.output = None
            self.writer = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SteeringPidRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            endpoint_future = node.remove_endpoint()
            if endpoint_future is not None:
                rclpy.spin_until_future_complete(
                    node, endpoint_future, timeout_sec=1.0
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
