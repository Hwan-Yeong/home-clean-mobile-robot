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

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped
from nav2_msgs.action import FollowPath
from tf2_ros import Buffer, TransformListener

# opennav_coverage action types
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates

from visualization_msgs.msg import Marker


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
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Visualization (created once, reused) ---
        self.marker_pub = self.create_publisher(Marker, '/coverage_path_markers', 10)

        self.get_logger().info(
            'CoverageClientNode started. Waiting for /cleaning_zone... '
            'Publish "true" on /clean_start to begin cleaning once ready '
            '(after setting the initial pose in RViz).'
        )

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
        """Handle coverage path result: trim it to start near the robot,
        sanitize it, and send it to FollowPath."""
        result = future.result().result

        if result.nav_path is None or len(result.nav_path.poses) == 0:
            self.get_logger().error('Coverage path is empty!')
            self.cleaning_in_progress = False
            return

        path = result.nav_path
        planning_time_sec = result.planning_time.sec + result.planning_time.nanosec * 1e-9

        if not path.header.frame_id:
            path.header.frame_id = 'map'

        xs = [p.pose.position.x for p in path.poses]
        ys = [p.pose.position.y for p in path.poses]
        self.get_logger().info(
            f'Coverage path computed with {len(path.poses)} waypoints '
            f'(planning time {planning_time_sec:.2f}s). '
            f'frame_id="{path.header.frame_id}", '
            f'x range [{min(xs):.2f}, {max(xs):.2f}], '
            f'y range [{min(ys):.2f}, {max(ys):.2f}]'
        )

        # --- Re-order the path so it starts near the robot's current pose ---
        # ComputeCoveragePath has no concept of a "start pose": Fields2Cover
        # always generates the same boustrophedon (snake) pattern for a given
        # polygon, with a fixed starting corner, regardless of where the
        # robot currently is. Without this step, FollowPath would be handed a
        # global plan whose first waypoints can be many meters away from
        # wherever the robot was placed via 2D Pose Estimate.
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is not None:
            robot_x, robot_y = robot_pose
            nearest_idx, nearest_dist = self.find_nearest_waypoint_index(path, robot_x, robot_y)

            self.get_logger().info(
                f'Robot pose in map frame: ({robot_x:.2f}, {robot_y:.2f}). '
                f'Nearest coverage waypoint: index {nearest_idx}/{len(path.poses) - 1} '
                f'at distance {nearest_dist:.2f}m.'
            )

            if nearest_idx > 0:
                self.get_logger().info(
                    f'Trimming {nearest_idx} earlier waypoint(s) so the path '
                    f'starts from the point closest to the robot.'
                )
                path.poses = path.poses[nearest_idx:]
        else:
            self.get_logger().warn(
                'Could not determine robot pose in the "map" frame (has the '
                'initial pose been set in RViz / is AMCL localized?). '
                'Sending the coverage path as-is; it may start far from '
                'the robot.'
            )

        if not path.poses:
            self.get_logger().error('Coverage path is empty after trimming!')
            self.cleaning_in_progress = False
            return

        # --- Sanitize headers/timestamps ---
        # FollowPath transforms each pose into the local costmap frame using
        # TF at the pose's timestamp. Use "now - 0.1s" so the timestamp is
        # always slightly in the past relative to the TF buffer (avoiding
        # extrapolation-into-the-future errors) while staying within
        # controller_server's transform_tolerance.
        stamp_msg = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        path.header.stamp = stamp_msg
        for pose in path.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = path.header.frame_id
            pose.header.stamp = stamp_msg

        # Fix invalid (all-zero) quaternions which can cause the
        # controller's plan transform/pruning to silently produce 0 poses.
        fixed_quat_count = 0
        for pose in path.poses:
            q = pose.pose.orientation
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                q.w = 1.0
                fixed_quat_count += 1

        if fixed_quat_count:
            self.get_logger().info(
                f'Fixed {fixed_quat_count} invalid (all-zero) quaternions in coverage path.'
            )

        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        self.get_logger().info(
            f'Sending coverage path with {len(path.poses)} waypoints. '
            f'start=({start.x:.2f}, {start.y:.2f}), end=({end.x:.2f}, {end.y:.2f})'
        )

        # Send path to Nav2 FollowPath
        self.follow_coverage_path(path)
        self.publish_path_markers(path)

    def get_robot_pose_in_map(self):
        """Return the robot's (x, y) position in the 'map' frame, or None.

        Requires AMCL to be localized (i.e. the initial pose has been set,
        e.g. via RViz's "2D Pose Estimate") so that the map->odom transform
        is available.
        """
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'map->base_link TF lookup failed: {e}')
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
