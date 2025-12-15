#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
import math
import json
import os
from typing import Optional


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics')
        
        # Parameters
        self.declare_parameter('waypoints_file', 'waypoints.json')
        self.declare_parameter('use_imu', True)
        
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        
        # Load robot parameters from waypoints.json metadata
        self.robot_radius = 0.1  # default
        self.marker_offset_x = 0.0
        self.marker_offset_y = 0.0
        self.load_robot_parameters()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Subscribers
        self.drawing_cmd_sub = self.create_subscription(
            Twist,
            '/drawing/cmd_vel',
            self.drawing_cmd_callback,
            1
        )
        
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/data',
                self.imu_callback,
                10
            )
        else:
            self.pose_sub = self.create_subscription(
                PoseStamped,
                '/robot_pose',
                self.pose_callback,
                10
            )
        
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )
        
        # State
        self.current_orientation = 0.0
        self.current_state = 'docked'
        self.active = False
        
        self.get_logger().info('Kinematics node initialized')
    
    def load_robot_parameters(self):
        """Load robot parameters from waypoints.json metadata."""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().warn(f'Waypoints file not found: {self.waypoints_file}, using defaults')
            return
        
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if 'metadata' in data:
                meta = data['metadata']
                self.robot_radius = float(meta.get('robot_radius_l', 0.1))
                self.marker_offset_x = float(meta.get('marker_offset_x', 0.0))
                self.marker_offset_y = float(meta.get('marker_offset_y', 0.0))
                self.get_logger().info(f'Loaded robot parameters: radius={self.robot_radius:.4f}, '
                                     f'offset=({self.marker_offset_x:.4f}, {self.marker_offset_y:.4f})')
        except Exception as e:
            self.get_logger().error(f'Error loading robot parameters: {e}')
    
    def state_callback(self, msg: String):
        """Handle robot state changes."""
        self.current_state = msg.data
        self.active = (self.current_state == 'drawing')
        if not self.active:
            # Stop robot when not in drawing state
            self.publish_stop()
    
    def imu_callback(self, msg: Imu):
        """Extract orientation from IMU."""
        # Convert quaternion to yaw
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.current_orientation = 2.0 * math.atan2(qz, qw)
    
    def pose_callback(self, msg: PoseStamped):
        """Extract orientation from pose."""
        # Convert quaternion to yaw
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_orientation = 2.0 * math.atan2(qz, qw)
    
    def drawing_cmd_callback(self, msg: Twist):
        """Convert world-frame velocity to wheel velocities."""
        if not self.active:
            return
        
        # Get world-frame velocities
        vx_world = -msg.linear.y
        vy_world = -msg.linear.x
        
        # Transform to body frame
        vx_body, vy_body = self.world_to_body_velocity(vx_world, vy_world, self.current_orientation)
        
        # Account for marker offset (if marker is moving, compute robot center velocity)
        # For now, assume velocities are already for robot center
        # TODO: Add marker offset compensation if needed
        
        # Compute wheel velocities using inverse kinematics
        v1, v2, v3 = self.inverse_kinematics(vx_body, vy_body, 0.0)  # omega = 0 for now
        
        # Publish wheel velocities
        wheel_cmd = Twist()
        wheel_cmd.linear.x = v1
        wheel_cmd.linear.y = v2
        wheel_cmd.linear.z = v3
        self.cmd_vel_pub.publish(wheel_cmd)
    
    def world_to_body_velocity(self, vx_world: float, vy_world: float, theta: float) -> tuple:
        """Transform world-frame velocity to body-frame velocity."""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        vx_body = cos_t * vx_world + sin_t * vy_world
        vy_body = -sin_t * vx_world + cos_t * vy_world
        return vx_body, vy_body
    
    def inverse_kinematics(self, vx_body: float, vy_body: float, omega: float) -> tuple:
        """Compute wheel velocities from body-frame velocities.
        
        Kiwi drive inverse kinematics:
        - Wheel 1: at 90 degrees (pointing up)
        - Wheel 2: at 210 degrees
        - Wheel 3: at 330 degrees
        """
        l = self.robot_radius
        sqrt3 = math.sqrt(3.0)
        
        v1 = vy_body + l * omega
        v2 = -0.5 * vy_body - (sqrt3 / 2.0) * vx_body + l * omega
        v3 = -0.5 * vy_body + (sqrt3 / 2.0) * vx_body + l * omega
        
        return v1, v2, v3
    
    def publish_stop(self):
        """Publish zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

