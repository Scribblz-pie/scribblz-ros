#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import os
import math
from typing import List, Dict, Optional, Sequence


class WaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        self.declare_parameter('waypoints_file', 'waypoints.json')
        self.declare_parameter('watch_file', True)
        self.declare_parameter('poll_interval', 1.0)
        
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.watch_file = self.get_parameter('watch_file').get_parameter_value().bool_value
        self.poll_interval = self.get_parameter('poll_interval').get_parameter_value().double_value
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/execute_drawing_path', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )
        
        # State tracking
        self.current_state = 'docked'
        self.waypoints_data = None
        self.last_file_mtime = 0.0
        self.current_phase = None  # 'undock', 'draw', 'dock', or None
        
        # Timer for file watching
        if self.watch_file:
            self.timer = self.create_timer(self.poll_interval, self.check_file)
        
        self.get_logger().info(f'Waypoint publisher initialized, watching: {self.waypoints_file}')
        
        # Try to load waypoints on startup
        self.load_waypoints()
    
    def state_callback(self, msg: String):
        """Handle robot state changes and publish appropriate phase."""
        old_state = self.current_state
        self.current_state = msg.data
        
        # Determine which phase to publish based on state transitions
        if old_state == 'docked' and self.current_state == 'drawing':
            # Starting drawing - publish undock + draw waypoints together
            # Control node will track through all waypoints sequentially
            self.current_phase = 'drawing'
            self.publish_combined_phases(['undock_waypoints', 'draw_waypoints'])
        elif self.current_state == 'returning_to_docking_station':
            # Returning to dock
            self.current_phase = 'dock'
            self.publish_phase('dock_waypoints')
    
    def check_file(self):
        """Check if waypoints file has been updated."""
        if not os.path.exists(self.waypoints_file):
            return
        
        try:
            mtime = os.path.getmtime(self.waypoints_file)
            if mtime > self.last_file_mtime:
                self.get_logger().info(f'Waypoints file updated, reloading...')
                self.load_waypoints()
                self.last_file_mtime = mtime
        except Exception as e:
            self.get_logger().error(f'Error checking file: {e}')
    
    def load_waypoints(self):
        """Load waypoints from JSON file."""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().warn(f'Waypoints file not found: {self.waypoints_file}')
            return False
        
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                self.waypoints_data = json.load(f)
            self.last_file_mtime = os.path.getmtime(self.waypoints_file)
            self.get_logger().info('Waypoints loaded successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return False
    
    def publish_phase(self, phase_key: str):
        """Publish waypoints for a specific phase."""
        if self.waypoints_data is None:
            self.get_logger().warn('No waypoints data loaded')
            return
        
        if phase_key not in self.waypoints_data:
            self.get_logger().warn(f'Phase {phase_key} not found in waypoints data')
            return
        
        waypoints = self.waypoints_data[phase_key]
        if not waypoints:
            self.get_logger().warn(f'No waypoints in phase {phase_key}')
            return
        
        # Convert to Path message
        path_msg = self.waypoints_to_path(waypoints, phase_key)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published {len(waypoints)} waypoints for phase {phase_key}')
    
    def publish_combined_phases(self, phase_keys: List[str]):
        """Publish waypoints from multiple phases combined into one path."""
        if self.waypoints_data is None:
            self.get_logger().warn('No waypoints data loaded')
            return
        
        all_waypoints = []
        for phase_key in phase_keys:
            if phase_key not in self.waypoints_data:
                self.get_logger().warn(f'Phase {phase_key} not found in waypoints data')
                continue
            
            waypoints = self.waypoints_data[phase_key]
            if waypoints:
                all_waypoints.extend(waypoints)
        
        if not all_waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
        
        # Convert to Path message
        path_msg = self.waypoints_to_path(all_waypoints, '+'.join(phase_keys))
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published {len(all_waypoints)} combined waypoints from phases: {phase_keys}')
    
    def waypoints_to_path(self, waypoints: List[Dict], phase_name: str) -> Path:
        """Convert JSON waypoints to nav_msgs/Path."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for wp in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path.header.stamp
            pose_stamped.header.frame_id = 'map'
            
            # Position
            pose_stamped.pose.position.x = float(wp['x'])
            pose_stamped.pose.position.y = float(wp['y'])
            pose_stamped.pose.position.z = 1.0 if wp.get('pen_down', False) else 0.0  # Store pen_down in z
            
            # Orientation (yaw to quaternion)
            yaw = float(wp.get('yaw', 0.0))
            pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            
            path.poses.append(pose_stamped)
        
        return path


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

