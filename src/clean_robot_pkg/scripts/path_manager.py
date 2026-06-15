import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CoveragePathManager:
    """
    Manages a coverage path by splitting it into distinct lanes (rows).
    Provides utilities to search for waypoints and handle lane transitions.
    """

    def __init__(self, logger):
        self.logger = logger
        self.raw_path = None
        self.lanes = []  # List of lists of PoseStamped
        self.current_lane_idx = 0
        self.lane_threshold_rad = math.radians(45.0)  # Threshold to detect a turn

    def set_path(self, path: Path):
        """Processes a new global coverage path."""
        self.raw_path = path
        self.lanes = self._split_into_lanes(path)
        self.current_lane_idx = 0
        self.logger.info(f"PathManager: Split path into {len(self.lanes)} lanes.")

    def _split_into_lanes(self, path: Path):
        """Splits a boustrophedon path into segments based on heading changes."""
        if not path.poses:
            return []

        lanes = []
        current_lane = []
        
        if len(path.poses) < 2:
            return [[path.poses[0]]]

        prev_heading = None
        
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            
            # Skip points that are too close to compute a stable heading
            if math.hypot(dx, dy) < 0.05:
                current_lane.append(path.poses[i])
                continue
                
            heading = math.atan2(dy, dx)
            
            if prev_heading is not None:
                # Calculate angular difference
                diff = heading - prev_heading
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                if abs(diff) > self.lane_threshold_rad:
                    # Detected a turn, finalize current lane
                    if current_lane:
                        lanes.append(current_lane)
                    current_lane = []
            
            current_lane.append(path.poses[i])
            prev_heading = heading
            
        current_lane.append(path.poses[-1])
        lanes.append(current_lane)
        
        # Filter out very short segments (noise)
        lanes = [l for l in lanes if len(l) > 2]
        
        return lanes

    def get_current_lane(self):
        if 0 <= self.current_lane_idx < len(self.lanes):
            return self.lanes[self.current_lane_idx]
        return None

    def find_nearest_waypoint_in_lane(self, lane_idx, robot_x, robot_y):
        """Finds the index of the nearest waypoint in a specific lane."""
        if lane_idx >= len(self.lanes):
            return None, None
            
        lane = self.lanes[lane_idx]
        best_idx = -1
        min_dist = float('inf')
        
        for i, pose in enumerate(lane):
            dist = math.hypot(pose.pose.position.x - robot_x, 
                              pose.pose.position.y - robot_y)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                
        return best_idx, min_dist

    def get_path_from_waypoint(self, lane_idx, start_wp_idx):
        """Returns a Path object starting from a specific waypoint in a lane."""
        if lane_idx >= len(self.lanes):
            return None
            
        new_path = Path()
        new_path.header = self.raw_path.header
        
        # Combine current lane and all subsequent lanes
        combined_poses = []
        combined_poses.extend(self.lanes[lane_idx][start_wp_idx:])
        
        for i in range(lane_idx + 1, len(self.lanes)):
            combined_poses.extend(self.lanes[i])
            
        new_path.poses = combined_poses
        return new_path

    def search_next_clear_waypoint(self, robot_x, robot_y, check_obstacle_func):
        """
        Searches for the next waypoint in the current lane that is clear.
        If the clear waypoint is > 2m away, moves to the next lane.
        """
        current_lane = self.get_current_lane()
        if not current_lane:
            return None, None

        # Find nearest point first to start searching from there
        near_idx, _ = self.find_nearest_waypoint_in_lane(self.current_lane_idx, robot_x, robot_y)
        
        # Search forward in current lane
        found_wp_idx = -1
        for i in range(near_idx, len(current_lane)):
            wp_x = current_lane[i].pose.position.x
            wp_y = current_lane[i].pose.position.y
            
            if not check_obstacle_func(wp_x, wp_y):
                found_wp_idx = i
                break
        
        if found_wp_idx != -1:
            dist = math.hypot(current_lane[found_wp_idx].pose.position.x - robot_x,
                              current_lane[found_wp_idx].pose.position.y - robot_y)
            
            if dist < 2.0:
                # Found a clear point in current lane within 2m
                return self.current_lane_idx, found_wp_idx
            else:
                self.logger.info(f"PathManager: Clear waypoint in lane {self.current_lane_idx} is {dist:.2f}m away (> 2m). Switching to next lane.")
        
        # If no clear point in current lane or too far, try next lane
        if self.current_lane_idx + 1 < len(self.lanes):
            self.current_lane_idx += 1
            # In the next lane, find the best entry point (nearest)
            next_near_idx, _ = self.find_nearest_waypoint_in_lane(self.current_lane_idx, robot_x, robot_y)
            self.logger.info(f"PathManager: Transitioned to lane {self.current_lane_idx}.")
            return self.current_lane_idx, next_near_idx
            
        return None, None
