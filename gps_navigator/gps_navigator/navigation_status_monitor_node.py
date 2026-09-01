import json
import math
import sys

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from mavros_msgs.msg import Mavlink, PositionTarget, RCOut
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
import yaml

from gps_navigator.geo_utils import haversine_distance, valid_nav_sat_fix
from gps_navigator.steering_pid_recorder_node import steering_pid_tuning


def progress_target_index(route_length, poses_remaining):
    """Return the route target index represented by Nav2 action feedback."""
    if route_length <= 0 or poses_remaining <= 0:
        return None
    return max(0, min(route_length - poses_remaining, route_length - 1))


def format_number(value, precision=2, signed=False):
    if value is None or not math.isfinite(value):
        return 'n/a'
    sign = '+' if signed else ''
    return f'{value:{sign}.{precision}f}'


def command_alert(nav_yaw_rate, smooth_yaw_rate, fcu_yaw_rate):
    values = (nav_yaw_rate, smooth_yaw_rate, fcu_yaw_rate)
    if any(value is None or not math.isfinite(value) for value in values):
        return None
    active = [value for value in values if abs(value) >= 0.03]
    if len(active) >= 2 and min(active) < 0.0 < max(active):
        return 'nav/smoother/fcu yaw-rate sign mismatch'
    return None


class NavigationStatusMonitor(Node):
    """Aggregate production telemetry into a compact operator dashboard."""

    def __init__(self):
        super().__init__('navigation_status_monitor')
        default_waypoints = (
            get_package_share_directory('gps_navigator')
            + '/config/waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_waypoints)
        self.declare_parameter(
            'position_topic', '/mavros/global_position/global'
        )
        self.declare_parameter(
            'heading_topic', '/mavros/global_position/compass_hdg'
        )
        self.declare_parameter('status_rate_hz', 1.0)
        self.declare_parameter('dashboard_mode', True)
        self.declare_parameter('steering_output_channel', 1)
        self.declare_parameter('throttle_output_channel', 3)

        rate = float(self.get_parameter('status_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('status_rate_hz must be positive')
        for parameter in (
            'steering_output_channel', 'throttle_output_channel'
        ):
            if self.get_parameter(parameter).value < 1:
                raise ValueError(f'{parameter} must be positive')

        with open(
            self.get_parameter('waypoints_file').value,
            encoding='utf-8',
        ) as stream:
            self.waypoints = yaml.safe_load(stream)['waypoints']

        self.route = []
        self.poses_remaining = None
        self.target_index = None
        self.latitude = None
        self.longitude = None
        self.heading_deg = None
        self.nav_speed = None
        self.nav_yaw_rate = None
        self.smooth_speed = None
        self.smooth_yaw_rate = None
        self.fcu_command_speed = None
        self.fcu_command_yaw_rate = None
        self.fcu_target_yaw_rate = None
        self.steering_pwm = None
        self.throttle_pwm = None
        self.pid_desired = None
        self.pid_achieved = None
        self.events = []
        self.navigation_state = 'WAITING'
        self.last_action_status = None

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            String, '/gps_navigation/route', self.route_callback, route_qos
        )
        self.create_subscription(
            NavigateThroughPoses.Impl.FeedbackMessage,
            '/navigate_through_poses/_action/feedback',
            self.feedback_callback,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_through_poses/_action/status',
            self.action_status_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            self.get_parameter('position_topic').value,
            self.gps_callback,
            sensor_qos,
        )
        self.create_subscription(
            Float64,
            self.get_parameter('heading_topic').value,
            self.heading_callback,
            sensor_qos,
        )
        self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_command_callback, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self.smooth_command_callback, 10
        )
        self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            self.fcu_command_callback,
            sensor_qos,
        )
        self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/target_local',
            self.fcu_target_callback,
            sensor_qos,
        )
        self.create_subscription(
            RCOut, '/mavros/rc/out', self.rc_out_callback, sensor_qos
        )
        self.create_subscription(
            Mavlink,
            '/mavros/mavlink/from',
            self.mavlink_callback,
            sensor_qos,
        )
        self.create_timer(1.0 / rate, self.log_status)

    @property
    def dashboard_mode(self):
        return bool(self.get_parameter('dashboard_mode').value)

    def add_event(self, text):
        event = f'[EVENT] {text}'
        if not self.events or self.events[-1] != event:
            self.events.append(event)
            self.events = self.events[-4:]
        if not self.dashboard_mode:
            self.get_logger().info(event)

    def route_callback(self, msg):
        try:
            route = json.loads(msg.data)['route']
            if not route or any(item not in self.waypoints for item in route):
                raise ValueError('empty route or unknown waypoint')
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'[NAV] Invalid route: {exc}')
            return
        self.route = route
        self.poses_remaining = None
        self.target_index = None
        self.navigation_state = 'READY'
        self.add_event(f'FIRST NODE: {route[0]}')
        self.add_event(f'ROUTE: {" -> ".join(route)}')

    def feedback_callback(self, msg):
        if not self.route:
            return
        remaining = int(msg.feedback.number_of_poses_remaining)
        new_target = progress_target_index(len(self.route), remaining)
        if self.poses_remaining is None:
            self.poses_remaining = remaining
            self.target_index = new_target
            self.log_following_event()
            return
        if remaining < self.poses_remaining:
            reached_count = min(
                self.poses_remaining - remaining,
                len(self.route),
            )
            first_reached = len(self.route) - self.poses_remaining
            for offset in range(reached_count):
                index = first_reached + offset
                if 0 <= index < len(self.route):
                    waypoint_id = self.route[index]
                    self.add_event(f'{waypoint_id} REACHED')
        self.poses_remaining = remaining
        if new_target != self.target_index:
            self.target_index = new_target
            self.log_following_event()

    def action_status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        if status == self.last_action_status:
            return
        self.last_action_status = status
        if status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
            self.navigation_state = 'RUNNING'
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.navigation_state = 'COMPLETED'
            self.add_event('NAVIGATION COMPLETED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.navigation_state = 'CANCELED'
            self.add_event('NAVIGATION CANCELED')
        elif status == GoalStatus.STATUS_ABORTED:
            self.navigation_state = 'FAILED'
            self.add_event('NAVIGATION FAILED')

    def log_following_event(self):
        if self.target_index is None:
            return
        target = self.route[self.target_index]
        target_name = self.waypoints[target]['name']
        if self.target_index == 0:
            self.add_event(f'TARGET: {target} ({target_name})')
            return
        previous = self.route[self.target_index - 1]
        self.add_event(f'TARGET: {previous} -> {target} ({target_name})')

    def route_edge_distance(self, start, end):
        a = self.waypoints[start]
        b = self.waypoints[end]
        return haversine_distance(
            a['latitude'], a['longitude'], b['latitude'], b['longitude']
        )

    def gps_callback(self, msg):
        if not valid_nav_sat_fix(msg):
            self.latitude = None
            self.longitude = None
            return
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def heading_callback(self, msg):
        self.heading_deg = msg.data if math.isfinite(msg.data) else None

    def nav_command_callback(self, msg):
        self.nav_speed = msg.linear.x
        self.nav_yaw_rate = msg.angular.z

    def smooth_command_callback(self, msg):
        self.smooth_speed = msg.linear.x
        self.smooth_yaw_rate = msg.angular.z

    def fcu_command_callback(self, msg):
        self.fcu_command_speed = msg.velocity.x
        self.fcu_command_yaw_rate = msg.yaw_rate

    def fcu_target_callback(self, msg):
        self.fcu_target_yaw_rate = msg.yaw_rate

    def rc_out_callback(self, msg):
        steering_index = (
            self.get_parameter('steering_output_channel').value - 1
        )
        throttle_index = (
            self.get_parameter('throttle_output_channel').value - 1
        )
        self.steering_pwm = (
            msg.channels[steering_index]
            if steering_index < len(msg.channels) else None
        )
        self.throttle_pwm = (
            msg.channels[throttle_index]
            if throttle_index < len(msg.channels) else None
        )

    def mavlink_callback(self, msg):
        tuning = steering_pid_tuning(msg)
        if tuning is None:
            return
        self.pid_desired = tuning['desired']
        self.pid_achieved = tuning['achieved']

    def target_distance(self):
        if (
            self.target_index is None
            or self.latitude is None
            or self.longitude is None
        ):
            return None
        target = self.waypoints[self.route[self.target_index]]
        return haversine_distance(
            self.latitude,
            self.longitude,
            target['latitude'],
            target['longitude'],
        )

    def segment_text(self):
        if self.target_index is None:
            return 'n/a', 'n/a'
        target = self.route[self.target_index]
        if self.target_index == 0:
            return f'start->{target}', target
        return f'{self.route[self.target_index - 1]}->{target}', target

    def log_status(self):
        if self.dashboard_mode:
            self.draw_dashboard()
            return
        segment, target = self.segment_text()
        gps = (
            'n/a' if self.latitude is None or self.longitude is None
            else f'{self.latitude:.7f},{self.longitude:.7f}'
        )
        self.get_logger().info(
            '[NAV STATUS] '
            f'segment={segment} next={target} '
            f'remaining={format_number(self.target_distance(), 1)}m '
            f'GPS=({gps}) yaw={format_number(self.heading_deg, 1)}deg'
        )
        self.get_logger().info(
            '[CONTROL] '
            f'nav=(v={format_number(self.nav_speed)},'
            f'w={format_number(self.nav_yaw_rate, signed=True)}) '
            f'smooth=(v={format_number(self.smooth_speed)},'
            f'w={format_number(self.smooth_yaw_rate, signed=True)}) '
            f'fcu_cmd=(v={format_number(self.fcu_command_speed)},'
            f'w={format_number(self.fcu_command_yaw_rate, signed=True)}) '
            'fcu_echo_w='
            f'{format_number(self.fcu_target_yaw_rate, signed=True)} '
            f'PID=(des={format_number(self.pid_desired, 1, True)},'
            f'ach={format_number(self.pid_achieved, 1, True)}deg/s) '
            f'steering_pwm={self.steering_pwm or "n/a"} '
            f'throttle_pwm={self.throttle_pwm or "n/a"}'
        )

    def draw_dashboard(self):
        route = ' -> '.join(self.route) if self.route else 'n/a'
        if self.target_index is None:
            target = 'n/a'
        else:
            waypoint_id = self.route[self.target_index]
            target = f'{waypoint_id} {self.waypoints[waypoint_id]["name"]}'
        gps = (
            'n/a' if self.latitude is None or self.longitude is None
            else f'{self.latitude:.7f}, {self.longitude:.7f}'
        )
        alerts = []
        yaw_alert = command_alert(
            self.nav_yaw_rate,
            self.smooth_yaw_rate,
            self.fcu_command_yaw_rate,
        )
        if yaw_alert:
            alerts.append(yaw_alert)
        if (
            self.steering_pwm is not None
            and (self.steering_pwm <= 1120 or self.steering_pwm >= 1880)
        ):
            alerts.append('steering PWM endpoint')
        lines = [
            '========== NAV2 LIVE ==========',
            f'ROUTE   {route}',
            f'TARGET  {target}',
            '',
            f'DIST    {format_number(self.target_distance(), 1)} m',
            f'GPS     {gps}',
            f'HDG     {format_number(self.heading_deg, 1)} deg',
            '',
            'CONTROL  NAV → SMOOTH → FCU',
            'NAV      v=%s  w=%s' % (
                format_number(self.nav_speed),
                format_number(self.nav_yaw_rate, signed=True),
            ),
            'SMOOTH   v=%s  w=%s' % (
                format_number(self.smooth_speed),
                format_number(self.smooth_yaw_rate, signed=True),
            ),
            'FCU      v=%s  w=%s' % (
                format_number(self.fcu_command_speed),
                format_number(self.fcu_command_yaw_rate, signed=True),
            ),
            '',
            f'STEER   {self.steering_pwm or "n/a"} PWM',
            f'THROT   {self.throttle_pwm or "n/a"} PWM',
            'PID      des=%s  ach=%s deg/s' % (
                format_number(self.pid_desired, 1),
                format_number(self.pid_achieved, 1),
            ),
            '',
            f'STATE   {self.navigation_state}',
            'ALERT   ' + ('; '.join(alerts) if alerts else 'none'),
            '',
            'RECENT EVENTS',
            *(self.events if self.events else ['n/a']),
        ]
        sys.stdout.write('\033[H\033[J' + '\n'.join(lines) + '\n')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
