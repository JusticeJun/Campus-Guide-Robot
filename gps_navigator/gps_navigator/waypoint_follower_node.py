import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

import math


# 목표 좌표
TARGET_LAT = 35.1351167
TARGET_LON = 129.1036015


STOP_DISTANCE = 5.0


class Navigator(Node):

    def __init__(self):
        super().__init__('gps_navigator')

        self.lat = None
        self.lon = None
        self.heading = None


        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT


        # GPS
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos
        )


        # Compass
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos
        )


        # ArduRover cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )


        self.timer = self.create_timer(
            0.2,
            self.control_loop
        )


    def gps_callback(self, msg):

        self.lat = msg.latitude
        self.lon = msg.longitude



    def heading_callback(self, msg):

        self.heading = msg.data



    # 거리 계산 (m)
    def calc_distance(self):

        R = 6371000

        lat1 = math.radians(self.lat)
        lat2 = math.radians(TARGET_LAT)

        dlat = math.radians(
            TARGET_LAT - self.lat
        )

        dlon = math.radians(
            TARGET_LON - self.lon
        )


        a = (
            math.sin(dlat / 2)**2
            +
            math.cos(lat1)
            *
            math.cos(lat2)
            *
            math.sin(dlon / 2)**2
        )


        c = 2 * math.atan2(
            math.sqrt(a),
            math.sqrt(1-a)
        )


        return R * c



    # 목표 방위각 계산
    def calc_bearing(self):

        lat1 = math.radians(self.lat)
        lat2 = math.radians(TARGET_LAT)

        dlon = math.radians(
            TARGET_LON - self.lon
        )


        y = math.sin(dlon) * math.cos(lat2)


        x = (
            math.cos(lat1)
            *
            math.sin(lat2)
            -
            math.sin(lat1)
            *
            math.cos(lat2)
            *
            math.cos(dlon)
        )


        bearing = math.degrees(
            math.atan2(y, x)
        )


        return (bearing + 360) % 360



    def control_loop(self):

        if self.lat is None:
            return

        if self.heading is None:
            return



        distance = self.calc_distance()

        target_heading = self.calc_bearing()


        error = target_heading - self.heading


        # -180 ~ 180 변환
        if error > 180:
            error -= 360

        if error < -180:
            error += 360



        cmd = Twist()



        # 목표 도착
        if distance <= STOP_DISTANCE:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)


            self.get_logger().info(
                f"도착! 거리:{distance:.1f}m 정지"
            )

            return



        # 항상 전진
        cmd.linear.x = 0.2


        # 방향 오차 보정
        cmd.angular.z = math.radians(error) * 0.5



        self.cmd_pub.publish(cmd)



        self.get_logger().info(
            f"거리:{distance:.1f}m "
            f"현재:{self.heading:.1f} "
            f"목표:{target_heading:.1f} "
            f"오차:{error:.1f} "
            f"속도:{cmd.linear.x:.2f} "
            f"회전:{cmd.angular.z:.2f}"
        )



def main(args=None):

    rclpy.init(args=args)

    node = Navigator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()