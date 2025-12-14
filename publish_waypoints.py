#!/usr/bin/env python3
"""
Publish waypoints directly to /execute_drawing_path topic.

Since the lidar is ON the docking station, waypoints are simply relative to the lidar origin.
- Use coordinates in meters relative to the dock/lidar position
- Example: (0.1, 0.2) = 0.1m right, 0.2m forward from dock

Usage:
  # From command line arguments (x y [yaw]):
  python3 publish_waypoints.py 0.1 0.2 0.3 0.4 0.5 0.6

  # With yaw angles (in radians):
  python3 publish_waypoints.py 0.1 0.2 0.0 0.3 0.4 1.57

  # From file (one waypoint per line: x y [yaw]):
  python3 publish_waypoints.py --file waypoints.txt

File format example:
  0.1 0.2 0.0
  0.3 0.4 1.57
  0.5 0.6 0.0
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


def yaw_to_quaternion(yaw):
    """Convert yaw angle (radians) to quaternion."""
    yaw_half = yaw / 2.0
    return [0.0, 0.0, math.sin(yaw_half), math.cos(yaw_half)]


class WaypointPublisher(Node):
    def __init__(self, waypoints):
        super().__init__('waypoint_publisher')
        self.publisher = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        # Create Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Add waypoints
        for wp in waypoints:
            x, y, yaw = wp
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            q = yaw_to_quaternion(float(yaw))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
        
        # Publish
        self.publisher.publish(path_msg)
        self.get_logger().info(f'Published path with {len(waypoints)} waypoints to /execute_drawing_path')
        
        # Give it a moment to publish
        rclpy.spin_once(self, timeout_sec=0.1)


def parse_waypoints_from_args(args):
    """Parse waypoints from command line arguments.
    
    Args format: x1 y1 [yaw1] x2 y2 [yaw2] ...
    If yaw is omitted, defaults to 0.0
    """
    waypoints = []
    i = 0
    while i < len(args):
        if i + 1 >= len(args):
            print(f'Error: Incomplete waypoint - need at least x and y')
            return None
        
        try:
            x = float(args[i])
            y = float(args[i + 1])
        except ValueError:
            print(f'Error: Invalid coordinates at argument {i}: {args[i]}, {args[i+1]}')
            return None
        
        # Check if yaw is provided (next arg is a number, not a flag)
        if i + 2 < len(args):
            # Check if it's a flag (starts with -)
            if args[i + 2].startswith('-'):
                yaw = 0.0
                i += 2
            else:
                try:
                    yaw = float(args[i + 2])
                    i += 3
                except ValueError:
                    yaw = 0.0
                    i += 2
        else:
            yaw = 0.0
            i += 2
        
        waypoints.append((x, y, yaw))
    
    return waypoints


def parse_waypoints_from_file(filename):
    """Parse waypoints from file.
    
    File format: one waypoint per line
    Each line: x y [yaw]
    If yaw is omitted, defaults to 0.0
    """
    waypoints = []
    try:
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                parts = line.split()
                if len(parts) < 2:
                    print(f'Warning: Skipping line {line_num}: {line}')
                    continue
                
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) > 2 else 0.0
                
                waypoints.append((x, y, yaw))
    except FileNotFoundError:
        print(f'Error: File not found: {filename}')
        return None
    except Exception as e:
        print(f'Error reading file: {e}')
        return None
    
    return waypoints


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    waypoints = None
    
    # Check if reading from file
    if sys.argv[1] == '--file' or sys.argv[1] == '-f':
        if len(sys.argv) < 3:
            print('Error: --file requires a filename')
            sys.exit(1)
        waypoints = parse_waypoints_from_file(sys.argv[2])
    else:
        # Parse from command line arguments
        waypoints = parse_waypoints_from_args(sys.argv[1:])
    
    if waypoints is None or len(waypoints) == 0:
        print('Error: No valid waypoints found')
        sys.exit(1)
    
    print(f'Parsed {len(waypoints)} waypoints:')
    for i, (x, y, yaw) in enumerate(waypoints):
        print(f'  {i+1}: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')
    
    rclpy.init()
    node = WaypointPublisher(waypoints)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

