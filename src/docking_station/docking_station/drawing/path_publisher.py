#!/usr/bin/env python3
"""
Simple path publisher node for testing path following.

Generates simple paths (line, square, etc.) and publishes them to /execute_drawing_path.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import json
import os


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Parameters
        self.declare_parameter('path_type', 'single')  # 'line', 'square', 'triangle', 'single', 'two_point', 'waypoint_file'
        self.declare_parameter('length', 0.5)  # Length of line or side of square/triangle
        self.declare_parameter('direction', 'forward')  # 'left', 'right', 'forward', 'backward'
        self.declare_parameter('num_waypoints', 10)  # Number of waypoints for line
        self.declare_parameter('target_x', 0.17268)  # Direct target X coordinate (second point for two_point)
        self.declare_parameter('target_y', 0.3546)  # Direct target Y coordinate (second point for two_point)
        self.declare_parameter('target_yaw', -90.0)  # Target orientation in degrees (second point for two_point)
        self.declare_parameter('start_x', -0.1174)  # Starting X position (first point for two_point)
        self.declare_parameter('start_y', 0.5243)  # Starting Y position (first point for two_point)
        self.declare_parameter('start_yaw', -90.0)  # Starting orientation in degrees (first point for two_point)
        self.declare_parameter('waypoint_file', '')  # Path to waypoint JSON file (for path_type='waypoint_file')
        self.declare_parameter('auto_publish', True)  # Publish immediately on startup
        self.declare_parameter('delay', 2.0)  # Delay before publishing (seconds)
        
        path_type = self.get_parameter('path_type').get_parameter_value().string_value
        length = self.get_parameter('length').get_parameter_value().double_value
        direction = self.get_parameter('direction').get_parameter_value().string_value
        num_waypoints = self.get_parameter('num_waypoints').get_parameter_value().integer_value
        target_x = self.get_parameter('target_x').get_parameter_value().double_value
        target_y = self.get_parameter('target_y').get_parameter_value().double_value
        target_yaw = self.get_parameter('target_yaw').get_parameter_value().double_value
        start_x = self.get_parameter('start_x').get_parameter_value().double_value
        start_y = self.get_parameter('start_y').get_parameter_value().double_value
        start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        auto_publish = self.get_parameter('auto_publish').get_parameter_value().bool_value
        delay = self.get_parameter('delay').get_parameter_value().double_value
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        # Track if we've already published to prevent multiple publications
        self.has_published = False
        
        self.get_logger().debug(f'Path publisher initialized: type={path_type}, length={length}m, direction={direction}, start=({start_x}, {start_y}), target=({target_x}, {target_y}, yaw={target_yaw}°)')
        
        if auto_publish:
            # Wait a bit for subscribers to connect, then publish once
            self.publish_timer = self.create_timer(delay, lambda: self.publish_path_once(path_type, length, direction, num_waypoints, target_x, target_y, target_yaw, start_x, start_y, start_yaw, waypoint_file))
    
    def publish_path_once(self, path_type, length, direction, num_waypoints, target_x, target_y, target_yaw, start_x, start_y, start_yaw, waypoint_file):
        """Publish path once and cancel timer."""
        if self.has_published:
            return
        
        waypoints = self.generate_path(path_type, length, direction, num_waypoints, target_x, target_y, target_yaw, start_x, start_y, start_yaw, waypoint_file)
        self.publish_path(waypoints)
        self.has_published = True
        
        # Cancel the timer to prevent it from firing again
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
    
    def load_waypoints_from_file(self, filepath):
        """Load waypoints from a JSON file.
        
        Args:
            filepath: Path to JSON file containing waypoints (can be absolute or relative)
            
        Returns:
            List of (x, y, yaw) tuples, where yaw is in radians
        """
        if not filepath:
            self.get_logger().error('No waypoint file specified')
            return []
        
        # Resolve path: if not absolute, try relative to package source directory
        if not os.path.isabs(filepath):
            # Get the package source directory (where waypoints/ folder is located)
            # __file__ is at: docking_station/drawing/path_publisher.py
            # We want to go up to: docking_station/ (package root)
            package_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            waypoints_dir = os.path.join(package_dir, 'waypoints')
            filepath = os.path.join(waypoints_dir, filepath)
        
        if not os.path.exists(filepath):
            self.get_logger().error(f'Waypoint file not found: {filepath}')
            return []
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            waypoints = []
            if 'waypoints' in data:
                for wp in data['waypoints']:
                    x = float(wp.get('x', 0.0))
                    y = float(wp.get('y', 0.0))
                    yaw_deg = float(wp.get('yaw', 0.0))
                    yaw_rad = math.radians(yaw_deg)  # Convert degrees to radians
                    marker = wp.get('marker', False)
                    waypoints.append((x, y, yaw_rad))
                    self.get_logger().debug(f'Loaded waypoint: ({x:.4f}, {y:.4f}, yaw={yaw_deg:.1f}°, marker={marker})')
            
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filepath}')
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoint file {filepath}: {e}')
            return []
    
    def generate_path(self, path_type, length, direction, num_waypoints, target_x, target_y, target_yaw, start_x, start_y, start_yaw, waypoint_file):
        """Generate waypoints for different path types.
        
        Args:
            target_yaw: Target orientation in degrees (converted to radians internally)
            start_x, start_y: Starting position offset
            start_yaw: Starting orientation in degrees
            waypoint_file: Path to waypoint JSON file (for path_type='waypoint_file')
        """
        waypoints = []
        target_yaw_rad = math.radians(target_yaw)  # Convert degrees to radians
        start_yaw_rad = math.radians(start_yaw)  # Convert degrees to radians
        
        if path_type == 'waypoint_file':
            # Load waypoints from JSON file
            waypoints = self.load_waypoints_from_file(waypoint_file)
            return waypoints
        
        elif path_type == 'line':
            # Generate a line path starting from (start_x, start_y)
            dx, dy = self.get_direction_vector(direction, length)
            
            if num_waypoints == 1:
                # Single waypoint at the end with target orientation
                waypoints.append((start_x + dx, start_y + dy, target_yaw_rad))
            else:
                # Multiple waypoints along the line
                # Interpolate orientation from start_yaw to target_yaw
                for i in range(num_waypoints):
                    t = i / (num_waypoints - 1) if num_waypoints > 1 else 0.0
                    x = start_x + dx * t
                    y = start_y + dy * t
                    yaw = start_yaw_rad + (target_yaw_rad - start_yaw_rad) * t  # Gradually rotate to target
                    waypoints.append((x, y, yaw))
        
        elif path_type == 'square':
            # Generate a square path starting from (start_x, start_y)
            # Square corners relative to start position
            side = length
            waypoints = [
                (start_x, start_y + side, start_yaw_rad + math.pi/2),    # Top-left, facing right
                (start_x + side, start_y + side, start_yaw_rad),         # Top-right, facing down
                (start_x + side, start_y, start_yaw_rad - math.pi/2),   # Bottom-right, facing left
                (start_x, start_y, start_yaw_rad + math.pi),             # Back to start, facing up
            ]
        
        elif path_type == 'triangle':
            # Equilateral triangle with side length "length", oriented by start_yaw
            side = length
            sqrt3_over_2 = math.sqrt(3) / 2.0

            # Base point
            p0 = (start_x, start_y)
            # Rotate helper
            def rot(dx, dy):
                cos_t, sin_t = math.cos(start_yaw_rad), math.sin(start_yaw_rad)
                return (dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t)

            # Second point along base direction
            dx1, dy1 = rot(side, 0.0)
            p1 = (p0[0] + dx1, p0[1] + dy1)

            # Third point (apex)
            dx2, dy2 = rot(-0.5 * side, sqrt3_over_2 * side)
            p2 = (p1[0] + dx2, p1[1] + dy2)

            # Helper to compute yaw toward next vertex
            def heading(a, b):
                return math.atan2(b[1] - a[1], b[0] - a[0])

            yaw0 = heading(p0, p1)
            yaw1 = heading(p1, p2)
            yaw2 = heading(p2, p0)

            waypoints = [
                (p0[0], p0[1], yaw0),
                (p1[0], p1[1], yaw1),
                (p2[0], p2[1], yaw2),
                (p0[0], p0[1], yaw0),  # close the loop
            ]

        elif path_type == 'single':
            # Single waypoint (for testing)
            # Use direct coordinates if provided, otherwise use direction/length from start
            if target_x != 0.0 or target_y != 0.0:
                waypoints.append((target_x, target_y, target_yaw_rad))
            else:
                dx, dy = self.get_direction_vector(direction, length)
                waypoints.append((start_x + dx, start_y + dy, target_yaw_rad))
        
        elif path_type == 'two_point':
            # Two waypoints at specific coordinates
            # Point 1: (start_x, start_y, start_yaw)
            # Point 2: (target_x, target_y, target_yaw)
            waypoints.append((start_x, start_y, start_yaw_rad))
            waypoints.append((target_x, target_y, target_yaw_rad))
        
        else:
            self.get_logger().error(f'Unknown path type: {path_type}')
            return []
        
        return waypoints
    
    def get_direction_vector(self, direction, length):
        """Get (dx, dy) vector for a direction."""
        directions = {
            'left': (0.0, length),
            'right': (0.0, -length),
            'forward': (length, 0.0),
            'backward': (-length, 0.0),
        }
        return directions.get(direction, (0.0, length))
    
    def publish_path(self, waypoints):
        """Publish a path from a list of (x, y, yaw) tuples."""
        if not waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion (simple case: only yaw rotation)
            # For yaw-only: q = (0, 0, sin(yaw/2), cos(yaw/2))
            half_yaw = yaw / 2.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().debug(f'Published path with {len(waypoints)} waypoints')
        for i, (x, y, yaw) in enumerate(waypoints):
            self.get_logger().debug(f'  Waypoint {i}: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
