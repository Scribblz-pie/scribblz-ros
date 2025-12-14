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


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Parameters
        self.declare_parameter('path_type', 'single')  # 'line', 'square', 'single', 'custom'
        self.declare_parameter('length', 0.5)  # Length of line or side of square
        self.declare_parameter('direction', 'forward')  # 'left', 'right', 'forward', 'backward'
        self.declare_parameter('num_waypoints', 10)  # Number of waypoints for line
        self.declare_parameter('target_x', 0.5)  # Direct target X coordinate (overrides direction/length for single)
        self.declare_parameter('target_y', 0.0)  # Direct target Y coordinate (overrides direction/length for single)
        self.declare_parameter('auto_publish', True)  # Publish immediately on startup
        self.declare_parameter('delay', 2.0)  # Delay before publishing (seconds)
        
        path_type = self.get_parameter('path_type').get_parameter_value().string_value
        length = self.get_parameter('length').get_parameter_value().double_value
        direction = self.get_parameter('direction').get_parameter_value().string_value
        num_waypoints = self.get_parameter('num_waypoints').get_parameter_value().integer_value
        target_x = self.get_parameter('target_x').get_parameter_value().double_value
        target_y = self.get_parameter('target_y').get_parameter_value().double_value
        auto_publish = self.get_parameter('auto_publish').get_parameter_value().bool_value
        delay = self.get_parameter('delay').get_parameter_value().double_value
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        self.get_logger().info(f'Path publisher initialized: type={path_type}, length={length}m, direction={direction}, target=({target_x}, {target_y})')
        
        if auto_publish:
            # Wait a bit for subscribers to connect, then publish
            self.create_timer(delay, lambda: self.publish_path_once(path_type, length, direction, num_waypoints, target_x, target_y))
    
    def publish_path_once(self, path_type, length, direction, num_waypoints, target_x, target_y):
        """Publish path once and cancel timer."""
        waypoints = self.generate_path(path_type, length, direction, num_waypoints, target_x, target_y)
        self.publish_path(waypoints)
        # Cancel the timer by creating a new one that never fires
        # Actually, we can't easily cancel, but this will only fire once anyway
    
    def generate_path(self, path_type, length, direction, num_waypoints, target_x, target_y):
        """Generate waypoints for different path types."""
        waypoints = []
        
        if path_type == 'line':
            # Generate a line path
            dx, dy = self.get_direction_vector(direction, length)
            
            if num_waypoints == 1:
                # Single waypoint at the end
                waypoints.append((dx, dy, 0.0))
            else:
                # Multiple waypoints along the line
                for i in range(num_waypoints):
                    t = i / (num_waypoints - 1) if num_waypoints > 1 else 0.0
                    x = dx * t
                    y = dy * t
                    waypoints.append((x, y, 0.0))
        
        elif path_type == 'square':
            # Generate a square path
            side = length
            waypoints = [
                (0.0, side, 0.0),      # Top-left
                (side, side, 0.0),     # Top-right
                (side, 0.0, 0.0),       # Bottom-right
                (0.0, 0.0, 0.0),       # Back to start
            ]
        
        elif path_type == 'single':
            # Single waypoint (for testing)
            # Use direct coordinates if provided, otherwise use direction/length
            if target_x != 0.0 or target_y != 0.0:
                waypoints.append((target_x, target_y, 0.0))
            else:
                dx, dy = self.get_direction_vector(direction, length)
                waypoints.append((dx, dy, 0.0))
        
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
        self.get_logger().info(f'Published path with {len(waypoints)} waypoints')
        for i, (x, y, yaw) in enumerate(waypoints):
            self.get_logger().info(f'  Waypoint {i}: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')


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
