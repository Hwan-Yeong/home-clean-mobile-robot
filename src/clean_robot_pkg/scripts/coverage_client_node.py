#!/usr/bin/env python3
"""
coverage_client_node.py
=======================
Client node that:
  1. Subscribes to /cleaning_zone (PolygonStamped) from map_to_polygon_node
     and keeps the most recent polygon in memory (does NOT auto-start).
  2. Waits for an explicit trigger on /clean_start (std_msgs/Bool, data=true).
  3. On trigger, calls the ComputeCoveragePath action to get a coverage path.
  4. Re-orders the resulting path so it starts at the waypoint closest to the
     robot's current pose (in the 'map' frame), since ComputeCoveragePath
     always generates the same pattern for a given polygon regardless of
     where the robot is.
  5. Sends the (trimmed) path to Nav2's FollowPath action for execution.

This node bridges opennav_coverage with standard Nav2 path following,
enabling indoor cleaning coverage on Humble (where CoverageNavigator
is not available).

Typical operating sequence:
  1. Launch the stack (04_coverage_clean.launch.py). The node will sit idle,
     logging that it is ready once a valid cleaning zone polygon is received.
  2. In RViz, set the initial pose (2D Pose Estimate) so AMCL localizes
     and map->odom TF becomes available. This pose determines where the
     coverage path will start.
  3. Trigger cleaning manually:
       ros2 topic pub /clean_start std_msgs/msg/Bool "{data: true}" --once

Subscribed Topics:
    /cleaning_zone (geometry_msgs/PolygonStamped)
    /clean_start   (std_msgs/Bool)  -- publish 'true' (once) to start cleaning

Published Topics:
    /coverage_path_markers (visualization_msgs/Marker) -- start/end arrows

Action Clients:
    /compute_coverage_path (opennav_coverage_msgs/action/ComputeCoveragePath)
    /follow_path (nav2_msgs/action/FollowPath)
"""

import sys
import os
import math

# Ensure local scripts can be imported safely
script_dir = os.path.dirname(os.path.realpath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

# OpenNav Coverage Action/Msg
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinates, Coordinate

# Nav2 Action
from nav2_msgs.action import FollowPath

# TF2
import tf2_ros
from tf2_ros import Buffer, TransformListener

# path_manager 안전 임포트
try:
    from path_manager import CoveragePathManager
except ImportError:
    from clean_robot_pkg.path_manager import CoveragePathManager

class CoverageClientNode(Node):
    """
    Orchestrates the complete cleaning coverage pipeline:
    polygon → coverage path (trimmed to robot pose) → follow path
    """

    def __init__(self):
        super().__init__('coverage_client_node')

        self.callback_group = ReentrantCallbackGroup()

        # State
        self.cleaning_in_progress = False
        self.latest_polygon = None       # Most recently received cleaning zone
        self._ready_logged = False       # Avoid spamming "ready" log every 1Hz

        # --- Subscribers ---
        self.polygon_sub = self.create_subscription(
            PolygonStamped,
            '/cleaning_zone',
            self.polygon_callback,
            10,
        )

        self.clean_start_sub = self.create_subscription(
            Bool,
            '/clean_start',
            self.clean_start_callback,
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

        # --- TF (used to find the robot's current pose in the 'map' frame,
        #     so the coverage path can be re-ordered to start near the
        #     robot's 2D Pose Estimate) ---
        self.tf_buffer = Buffer()
        # Create a dedicated callback group for TF to avoid interference
        self.tf_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- Path Management ---
        self.path_manager = CoveragePathManager(self.get_logger())
        self.transitioning = False  # To avoid redundant transition calls
        self.scan_data = None
        self.obstacle_detected = False

        # --- Subscribers ---
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.marker_pub = self.create_publisher(Marker, '/coverage_path_markers', 10)

        # --- Obstacle Monitoring Timer ---
        self.monitor_timer = self.create_timer(0.5, self.obstacle_monitor_timer_callback)

        self.get_logger().info(
            'CoverageClientNode started. Waiting for /cleaning_zone... '
            'Publish "true" on /clean_start to begin cleaning once ready '
            '(after setting the initial pose in RViz).'
        )

    # ------------------------------------------------------------------
    # Sensor Callbacks & Monitoring
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """Monitor laser scan for obstacles in front of the robot."""
        self.scan_data = msg
        
        # Simple front obstacle detection: 
        # Check within +/- 15 degrees, distance < 0.6m
        # robot front is around index 0 (depending on lidar orientation)
        # For TB3 waffle, 0 is front, ranges[0:15] and ranges[345:359]
        angle_range = 15
        front_ranges = []
        
        # Handles wrap around
        for i in range(-angle_range, angle_range + 1):
            idx = i % len(msg.ranges)
            if msg.range_min < msg.ranges[idx] < msg.range_max:
                front_ranges.append(msg.ranges[idx])
        
        if front_ranges:
            min_front_dist = min(front_ranges)
            if min_front_dist < 0.6: # 0.6m threshold
                if not self.obstacle_detected:
                    self.get_logger().info(f"Obstacle detected! Min distance: {min_front_dist:.2f}m")
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def obstacle_monitor_timer_callback(self):
        """Periodically check if we should trigger a path transition."""
        if not self.cleaning_in_progress or self.transitioning:
            return

        if self.obstacle_detected:
            self.get_logger().info("Obstacle blocking path. Initiating path transition algorithm...")
            self.perform_path_transition()

    def perform_path_transition(self):
        """Cancels current path and searches for the next best waypoint."""
        self.transitioning = True
        
        # Stop current movement
        self.get_logger().info("Cancelling current goal...")
        self.follow_path_client.cancel_all_goals()
        
        # Get current pose
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is None:
            self.get_logger().error("Could not get robot pose for transition!")
            self.transitioning = False
            return
            
        robot_x, robot_y = robot_pose
        
        # Simple placeholder for obstacle check at waypoint: 
        # In a real scenario, we might use costmap. For now, we assume current 
        # position is the only one blocked, and we search for clear space.
        # But per user request, we search for clear waypoint and check 2m distance.
        
        def dummy_check_wp_blocked(x, y):
            # If waypoint is very close to current robot front obstacle, consider it blocked
            # This is a simplification.
            dist_to_robot = math.hypot(x - robot_x, y - robot_y)
            if dist_to_robot < 0.5: # Waypoint too close to current blocked spot
                return True
            return False

        lane_idx, wp_idx = self.path_manager.search_next_clear_waypoint(
            robot_x, robot_y, dummy_check_wp_blocked)
        
        if lane_idx is not None:
            new_path = self.path_manager.get_path_from_waypoint(lane_idx, wp_idx)
            if new_path:
                self.get_logger().info(f"Transition successful. Resuming from lane {lane_idx}, waypoint {wp_idx}.")
                self.send_updated_path(new_path)
            else:
                self.get_logger().error("Failed to generate updated path for transition.")
                self.cleaning_in_progress = False
        else:
            self.get_logger().warn("No clear waypoints found in current or next lanes. Cleaning halted.")
            self.cleaning_in_progress = False
            
        self.transitioning = False

    def send_updated_path(self, path):
        """Sanitize and send a new path to FollowPath."""
        stamp_msg = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        path.header.stamp = stamp_msg
        for pose in path.poses:
            pose.header.stamp = stamp_msg
            pose.header.frame_id = path.header.frame_id
            if pose.pose.orientation.w == 0.0:
                 pose.pose.orientation.w = 1.0

        self.follow_coverage_path(path)
        self.publish_path_markers(path)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def polygon_callback(self, msg: PolygonStamped):
        """Receive cleaning zone polygon and keep the latest one in memory.

        Does NOT start cleaning automatically. Cleaning is triggered
        separately via /clean_start.
        """
        if len(msg.polygon.points) < 3:
            self.get_logger().warn('Polygon has fewer than 3 points, skipping.')
            return

        self.latest_polygon = msg

        if not self._ready_logged:
            self._ready_logged = True
            self.get_logger().info(
                f'Cleaning zone ready ({len(msg.polygon.points)} vertices). '
                'Set the initial pose in RViz if you have not already, then '
                'publish "true" on /clean_start to begin cleaning, e.g.:\n'
                '  ros2 topic pub /clean_start std_msgs/msg/Bool '
                '"{data: true}" --once'
            )

    def clean_start_callback(self, msg: Bool):
        """Trigger the cleaning pipeline when /clean_start receives true."""
        if not msg.data:
            return

        if self.cleaning_in_progress:
            self.get_logger().warn(
                'Received /clean_start, but cleaning is already in progress. Ignoring.'
            )
            return

        if self.latest_polygon is None:
            self.get_logger().warn(
                'Received /clean_start, but no cleaning zone polygon has been '
                'received yet on /cleaning_zone. Ignoring.'
            )
            return

        self.get_logger().info('Received /clean_start trigger. Beginning coverage cleaning.')
        self.start_coverage(self.latest_polygon)

    # ------------------------------------------------------------------
    # ComputeCoveragePath
    # ------------------------------------------------------------------
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

        boundary = Coordinates()
        boundary.coordinates = coordinates
        goal.polygons = [boundary]

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
        """Handle coverage path result, initialize PathManager, and start following."""
        result = future.result().result

        if result.nav_path is None or len(result.nav_path.poses) == 0:
            self.get_logger().error('Coverage path is empty!')
            self.cleaning_in_progress = False
            return

        path = result.nav_path
        
        # Initialize PathManager with the new path
        self.path_manager.set_path(path)
        
        # Trim initial waypoints to start near robot
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is not None:
            robot_x, robot_y = robot_pose
            lane_idx, wp_idx = self.path_manager.find_nearest_waypoint_in_lane(0, robot_x, robot_y)
            
            # Update PathManager to current lane and state
            # For simplicity, start from lane 0 if near.
            # (In a more robust version, we'd search all lanes to find the truly nearest)
            
            trimmed_path = self.path_manager.get_path_from_waypoint(0, lane_idx if lane_idx else 0)
            if trimmed_path:
                self.send_updated_path(trimmed_path)
            else:
                self.get_logger().error("Failed to start path following after trimming.")
                self.cleaning_in_progress = False
        else:
            self.get_logger().warn("Robot pose unknown. Starting from full coverage path.")
            self.send_updated_path(path)

    def get_robot_pose_in_map(self):
        """Return the robot's (x, y) position in the 'map' frame, or None.

        Requires AMCL to be localized (i.e. the initial pose has been set,
        e.g. via RViz's "2D Pose Estimate") so that the map->odom transform
        is available.
        """
        try:
            # Use current time or 0 for latest available
            now = Time()
            
            # Wait a tiny bit if needed, especially after initial pose set
            tf = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                now,
                timeout=Duration(seconds=0.1)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'map->base_link TF lookup failed (normal during init/jump): {e}')
            return None

    @staticmethod
    def find_nearest_waypoint_index(path, x, y):
        """Return (index, distance) of the path pose closest to (x, y)."""
        best_idx = 0
        best_dist = math.inf
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        return best_idx, best_dist

    # ------------------------------------------------------------------
    # FollowPath
    # ------------------------------------------------------------------
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

    def publish_path_markers(self, path):
        """Publish start (green) / end (red) arrow markers for the path."""
        for i, (color, idx) in enumerate([
            ((0.0, 1.0, 0.0, 1.0), 0),
            ((1.0, 0.0, 0.0, 1.0), -1),
        ]):
            marker = Marker()
            marker.header.frame_id = path.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = path.poses[idx].pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            self.marker_pub.publish(marker)

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
