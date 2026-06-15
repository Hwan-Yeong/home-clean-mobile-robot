#!/usr/bin/env python3
"""
map_to_polygon_node.py
======================
Subscribes to the /map topic (nav_msgs/OccupancyGrid),
extracts the largest free-space contour using OpenCV,
and publishes it as a geometry_msgs/PolygonStamped.

This polygon is used by the coverage_client_node to request
coverage path planning from the opennav_coverage server.

Published Topics:
    /cleaning_zone (geometry_msgs/PolygonStamped)
        The polygon representing the cleaning area in map frame.

Subscribed Topics:
    /map (nav_msgs/OccupancyGrid)
        The occupancy grid from map_server or SLAM.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point32


class MapToPolygonNode(Node):
    """
    Convert an OccupancyGrid map to a polygon that represents
    the largest free-space region for coverage planning.
    """

    def __init__(self):
        super().__init__('map_to_polygon_node')

        if not HAS_CV2:
            self.get_logger().error(
                'OpenCV (cv2) is not installed. '
                'Please install: sudo apt install python3-opencv'
            )
            return

        # Parameters
        self.declare_parameter('free_threshold', 50)
        self.declare_parameter('contour_simplify_epsilon', 0.02)
        self.declare_parameter('min_area_ratio', 0.05)
        self.declare_parameter('publish_rate', 1.0)

        self.free_threshold = self.get_parameter('free_threshold').value
        self.simplify_epsilon = self.get_parameter('contour_simplify_epsilon').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value

        # QoS for map topic (transient local, reliable)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        # Subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        # Publisher
        self.polygon_pub = self.create_publisher(
            PolygonStamped,
            '/cleaning_zone',
            10,
        )

        self.latest_polygon = None

        # Periodic publisher (for late subscribers)
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_polygon)

        self.get_logger().info('MapToPolygonNode started. Waiting for /map...')

    def map_callback(self, msg: OccupancyGrid):
        """Process an occupancy grid and extract the free-space polygon."""
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution}'
        )

        # Convert OccupancyGrid to numpy array
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # OccupancyGrid data: 0=free, 100=occupied, -1=unknown
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create binary image: free space = 255 (white), rest = 0 (black)
        # Values of 0 (free) and small values are considered free space
        free_mask = np.zeros((height, width), dtype=np.uint8)
        free_mask[grid >= 0] = 255  # Known cells
        free_mask[grid > self.free_threshold] = 0  # Occupied cells
        free_mask[grid < 0] = 0  # Unknown cells

        # Morphological operations to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
        free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            self.get_logger().warn('No free-space contours found in map!')
            return

        # Find the largest contour (main cleaning area)
        largest_contour = max(contours, key=cv2.contourArea)
        total_area = width * height
        contour_area = cv2.contourArea(largest_contour)

        if contour_area / total_area < self.min_area_ratio:
            self.get_logger().warn(
                f'Largest contour too small: {contour_area / total_area:.2%} of total area'
            )
            return

        # Simplify the contour to reduce the number of points
        perimeter = cv2.arcLength(largest_contour, True)
        simplified = cv2.approxPolyDP(
            largest_contour,
            self.simplify_epsilon * perimeter,
            True,
        )

        # Convert pixel coordinates to map coordinates (meters)
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = 'map'
        polygon_msg.header.stamp = self.get_clock().now().to_msg()

        for point in simplified:
            px, py = point[0]

            # Convert from pixel (col, row) to world coordinates
            # Note: OpenCV y-axis is inverted relative to map frame
            world_x = origin_x + px * resolution
            world_y = origin_y + (height - py) * resolution

            p = Point32()
            p.x = float(world_x)
            p.y = float(world_y)
            p.z = 0.0
            polygon_msg.polygon.points.append(p)

        # Close the polygon: Fields2Cover requires first == last point
        if len(polygon_msg.polygon.points) > 0:
            first = polygon_msg.polygon.points[0]
            last = polygon_msg.polygon.points[-1]
            if first.x != last.x or first.y != last.y:
                closing = Point32()
                closing.x = first.x
                closing.y = first.y
                closing.z = first.z
                polygon_msg.polygon.points.append(closing)

        self.latest_polygon = polygon_msg

        self.get_logger().info(
            f'Extracted cleaning zone polygon with {len(simplified)} vertices, '
            f'area ratio: {contour_area / total_area:.2%}'
        )

    def publish_polygon(self):
        """Periodically publish the polygon for late subscribers."""
        if self.latest_polygon is not None:
            self.latest_polygon.header.stamp = self.get_clock().now().to_msg()
            self.polygon_pub.publish(self.latest_polygon)


def main(args=None):
    rclpy.init(args=args)
    node = MapToPolygonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()