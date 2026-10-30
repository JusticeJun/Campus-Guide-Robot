"""Buffer only stored obstacles; live scan obstacles keep the smaller footprint."""

from array import array
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def buffered_cells(data, width, height, resolution, margin):
    """
    Conservative disk dilation of occupied cells, preserving unknown cells.

    A one-cell guard covers rasterization of the requested radial margin.
    Work is uint8/int8, not a full floating-point distance transform.
    """
    if (width <= 0 or height <= 0 or len(data) != width * height
            or not math.isfinite(resolution) or resolution <= 0
            or not math.isfinite(margin) or margin < 0):
        raise ValueError('Invalid static map geometry or clearance')
    cells = np.asarray(data, dtype=np.int8).reshape(height, width)
    if margin == 0:
        return cells.copy()
    radius = math.ceil(margin / resolution - 1e-9) + 1
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    kernel = (xx * xx + yy * yy <= radius * radius).astype(np.uint8)
    marked = cv2.dilate((cells >= 100).astype(np.uint8), kernel)
    result = cells.copy()
    result[(marked != 0) & (cells >= 0)] = 100
    return result


class StaticClearance(Node):
    def __init__(self):
        super().__init__('static_clearance')
        cv2.setNumThreads(1)
        self.margin = float(self.declare_parameter('extra_margin_m', 0.0).value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            OccupancyGrid, '/navigation/static_clearance_map', qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)

    def on_map(self, msg):
        try:
            cells = buffered_cells(msg.data, msg.info.width, msg.info.height,
                                   msg.info.resolution, self.margin)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        output = OccupancyGrid()
        output.header, output.info = msg.header, msg.info
        output.data = array('b', cells.reshape(-1).tobytes())
        self.publisher.publish(output)
        self.get_logger().info(f'Stored-obstacle margin ready: {self.margin:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = StaticClearance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
