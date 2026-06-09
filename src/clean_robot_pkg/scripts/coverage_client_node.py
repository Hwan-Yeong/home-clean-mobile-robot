#!/usr/bin/env python3
"""
coverage_client_node.py
=======================
Client node that:
  1. Subscribes to /cleaning_zone (PolygonStamped) from map_to_polygon_node
  2. Calls the ComputeCoveragePath action to get coverage path
  3. Sends the resulting path to Nav2's FollowPath action for execution

This node bridges opennav_coverage with standard Nav2 path following,
enabling indoor cleaning coverage on Humble (where CoverageNavigator
is not available).

Subscribed Topics:
    /cleaning_zone (geometry_msgs/PolygonStamped)

Action Clients:
    /compute_coverage_path (opennav_coverage_msgs/action/ComputeCoveragePath)
    /follow_path (nav2_msgs/action/FollowPath)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PolygonStamped
from nav2_msgs.action import FollowPath

# opennav_coverage action types
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate


class CoverageClientNode(Node):
    """
    Orchestrates the complete cleaning coverage pipeline:
    polygon → coverage path → follow path
    """

    def __init__(self):
        super().__init__('coverage_client_node')

        self.callback_group = ReentrantCallbackGroup()

        # State
        self.cleaning_in_progress = False
        self.polygon_received = False

        # --- Subscribers ---
        self.polygon_sub = self.create_subscription(
            PolygonStamped,
            '/cleaning_zone',
            self.polygon_callback,
            10,
        )

        # --- Action Clients ---
        self.coverage_client = ActionClient(
            self,
            ComputeCoveragePath,
            '/compute_coverage_path',
            callback_group=self.callback_group,
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            '/follow_path',
            callback_group=self.callback_group,
        )

        self.get_logger().info('CoverageClientNode started. Waiting for /cleaning_zone...')

        # Wait for action servers
        self.get_logger().info('Waiting for coverage and follow_path action servers...')

    def polygon_callback(self, msg: PolygonStamped):
        """Receive cleaning zone polygon and start coverage if not already running."""
        if self.polygon_received:
            return  # Only process the first polygon

        if len(msg.polygon.points) < 3:
            self.get_logger().warn('Polygon has fewer than 3 points, skipping.')
            return

        self.polygon_received = True
        self.get_logger().info(
            f'Received cleaning zone with {len(msg.polygon.points)} vertices.'
        )

        # Start the cleaning pipeline
        self.start_coverage(msg)

    def start_coverage(self, polygon_msg: PolygonStamped):
        """Send the polygon to the coverage server."""
        if self.cleaning_in_progress:
            self.get_logger().warn('Cleaning already in progress!')
            return

        self.get_logger().info('Waiting for ComputeCoveragePath action server...')
        if not self.coverage_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('ComputeCoveragePath action server not available!')
            return

        # Build the coverage goal
        goal = ComputeCoveragePath.Goal()
        goal.use_gml_file = False
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = polygon_msg.header.frame_id

        # Convert PolygonStamped points to opennav_coverage Coordinate format
        coordinates = []
        for point in polygon_msg.polygon.points:
            coord = Coordinate()
            coord.axis1 = float(point.x)
            coord.axis2 = float(point.y)
            coordinates.append(coord)

        goal.polygons = [coordinates]

        self.get_logger().info(
            f'Sending coverage request with {len(coordinates)} vertices...'
        )
        self.cleaning_in_progress = True

        future = self.coverage_client.send_goal_async(
            goal,
            feedback_callback=self.coverage_feedback_callback,
        )
        future.add_done_callback(self.coverage_goal_response_callback)

    def coverage_feedback_callback(self, feedback_msg):
        """Handle coverage planning feedback."""
        pass  # Coverage planning is typically fast, no significant feedback

    def coverage_goal_response_callback(self, future):
        """Handle coverage goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Coverage goal was rejected!')
            self.cleaning_in_progress = False
            return

        self.get_logger().info('Coverage goal accepted. Computing path...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.coverage_result_callback)

    def coverage_result_callback(self, future):
        """Handle coverage path result and send to FollowPath."""
        result = future.result().result

        if result.nav_path is None or len(result.nav_path.poses) == 0:
            self.get_logger().error('Coverage path is empty!')
            self.cleaning_in_progress = False
            return

        path = result.nav_path
        self.get_logger().info(
            f'Coverage path computed with {len(path.poses)} waypoints. '
            f'Planning time: {result.planning_time:.2f}s'
        )

        # Send path to Nav2 FollowPath
        self.follow_coverage_path(path)

    def follow_coverage_path(self, path):
        """Send the coverage path to Nav2 FollowPath action."""
        self.get_logger().info('Waiting for FollowPath action server...')
        if not self.follow_path_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('FollowPath action server not available!')
            self.cleaning_in_progress = False
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''  # Use default controller
        goal.goal_checker_id = ''  # Use default goal checker

        self.get_logger().info(
            f'Starting coverage path following ({len(path.poses)} waypoints)...'
        )

        future = self.follow_path_client.send_goal_async(
            goal,
            feedback_callback=self.follow_path_feedback_callback,
        )
        future.add_done_callback(self.follow_path_goal_response_callback)

    def follow_path_feedback_callback(self, feedback_msg):
        """Handle FollowPath feedback (distance remaining, ETA, etc.)."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Cleaning progress - Distance remaining: {feedback.distance_to_goal:.2f}m',
            throttle_duration_sec=5.0,
        )

    def follow_path_goal_response_callback(self, future):
        """Handle FollowPath goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected!')
            self.cleaning_in_progress = False
            return

        self.get_logger().info('FollowPath goal accepted. Robot is cleaning...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result_callback)

    def follow_path_result_callback(self, future):
        """Handle FollowPath completion."""
        result = future.result()

        self.cleaning_in_progress = False

        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(
                '========================================\n'
                '  🎉 CLEANING COMPLETE!\n'
                '  Coverage path successfully followed.\n'
                '========================================'
            )
        else:
            self.get_logger().warn(
                f'FollowPath ended with status: {result.status}. '
                'Cleaning may be incomplete.'
            )

        # Allow re-triggering by resetting
        self.polygon_received = False


def main(args=None):
    rclpy.init(args=args)
    node = CoverageClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
