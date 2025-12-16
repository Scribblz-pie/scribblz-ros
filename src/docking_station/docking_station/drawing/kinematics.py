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
        self.robot_radius = .174  # default
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
        
        self.get_logger().debug('Kinematics node initialized')
    
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
                self.get_logger().debug(f'Loaded robot parameters: radius={self.robot_radius:.4f}, '
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
        """Convert body-frame velocity to wheel velocities.
        
        Path follower now outputs body-frame velocities directly (x=forward, y=left),
        so no transform is needed. Just apply inverse kinematics.
        """
        if not self.active:
            return
        
        # Get body-frame velocities directly from path_follower
        # x = forward, y = left (lateral), z = angular
        vx_body = msg.linear.x  # Forward velocity
        vy_body = msg.linear.y  # Lateral velocity (left)
        omega = msg.angular.z   # Angular velocity
        
        # Debug: Log body frame velocities (input to inverse kinematics)
        self.get_logger().debug(
            f'[KINEMATICS INPUT] Body frame velocities: '
            f'vx={vx_body:.3f}m/s, vy={vy_body:.3f}m/s, omega={omega:.3f}rad/s'
        )
        
        # Account for marker offset (if marker is moving, compute robot center velocity)
        # For now, assume velocities are already for robot center
        # TODO: Add marker offset compensation if needed
        
        # Compute wheel velocities using inverse kinematics
        v1, v2, v3 = self.inverse_kinematics(vx_body, vy_body, omega)
        
        # Debug: Log wheel velocities (output from inverse kinematics)
        self.get_logger().debug(
            f'[KINEMATICS OUTPUT] Wheel velocities: '
            f'v1={v1:.3f}m/s, v2={v2:.3f}m/s, v3={v3:.3f}m/s'
        )
        
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
        
        Note: Input follows ROS convention (+Y = left), but physical robot wiring
        has +Y = right, so we negate vy_body to compensate.
        """
        l = self.robot_radius
        sqrt3 = math.sqrt(3.0)
        
        # Negate vy to convert ROS convention (+Y = left) to physical wiring (+Y = right)
        vy = -vy_body
        
        v1 = vy + l * omega
        v2 = -0.5 * vy - (sqrt3 / 2.0) * vx_body + l * omega
        v3 = -0.5 * vy + (sqrt3 / 2.0) * vx_body + l * omega
        
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

