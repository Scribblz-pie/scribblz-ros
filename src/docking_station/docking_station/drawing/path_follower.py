#!/usr/bin/env python3
"""
Path follower node for kiwi drive robot.

Implements high-level path following by:
1. Computing position and heading errors in global frame
2. Transforming errors to robot body frame
3. Using PID control to compute desired chassis velocities
4. Publishing chassis velocities (kinematics node converts to wheel velocities)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Bool, UInt32, Float64
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
import math
from typing import Optional


class PathFollowerNode(Node):
    """Path follower that converts waypoint errors to chassis velocity commands."""
    
    def __init__(self):
        super().__init__('path_follower')
        
        # Declare parameters
        self.declare_parameter('waypoint_tolerance', 0.05)  # Distance threshold (m)
        self.declare_parameter('heading_tolerance', 0.1)  # Heading threshold (rad)
        self.declare_parameter('max_forward_vel', 0.3)  # Max forward velocity (m/s)
        self.declare_parameter('max_lateral_vel', 0.3)  # Max lateral velocity (m/s)
        self.declare_parameter('max_angular_vel', 0.5)  # Max angular velocity (rad/s)
        self.declare_parameter('control_frequency', 20.0)  # Control loop frequency (Hz)
        
        # PID gains for position control (in body frame)
        self.declare_parameter('kp_forward', 2.0)
        self.declare_parameter('ki_forward', 0.0)
        self.declare_parameter('kd_forward', 0.0)
        
        self.declare_parameter('kp_lateral', 2.0)
        self.declare_parameter('ki_lateral', 0.0)
        self.declare_parameter('kd_lateral', 0.0)
        
        # PID gains for heading control
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('ki_heading', 0.0)
        self.declare_parameter('kd_heading', 0.0)
        
        # Get parameter values
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.heading_tolerance = self.get_parameter('heading_tolerance').get_parameter_value().double_value
        self.max_forward_vel = self.get_parameter('max_forward_vel').get_parameter_value().double_value
        self.max_lateral_vel = self.get_parameter('max_lateral_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        self.kp_forward = self.get_parameter('kp_forward').get_parameter_value().double_value
        self.ki_forward = self.get_parameter('ki_forward').get_parameter_value().double_value
        self.kd_forward = self.get_parameter('kd_forward').get_parameter_value().double_value
        
        self.kp_lateral = self.get_parameter('kp_lateral').get_parameter_value().double_value
        self.ki_lateral = self.get_parameter('ki_lateral').get_parameter_value().double_value
        self.kd_lateral = self.get_parameter('kd_lateral').get_parameter_value().double_value
        
        self.kp_heading = self.get_parameter('kp_heading').get_parameter_value().double_value
        self.ki_heading = self.get_parameter('ki_heading').get_parameter_value().double_value
        self.kd_heading = self.get_parameter('kd_heading').get_parameter_value().double_value
        
        # Initialize state
        self.current_state = 'docked'
        self.active = False
        self.executing = False
        self.cancel_requested = False
        
        # Current path and waypoint tracking
        self.current_path: Optional[Path] = None
        self.stored_path: Optional[Path] = None
        self.current_waypoint_idx = 0
        
        # Current pose (from sensors)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_pose = False
        
        # PID state
        self.dt = 1.0 / self.control_freq
        self.prev_forward_error = 0.0
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_forward = 0.0
        self.integral_lateral = 0.0
        self.integral_heading = 0.0
        
        # Setup subscribers
        self.path_sub = self.create_subscription(
            Path, '/execute_drawing_path', self.path_callback, 1)
        
        self.cancel_sub = self.create_subscription(
            Empty, '/cancel_drawing', self.cancel_callback, 1)
        
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Setup publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 1)
        self.progress_pub = self.create_publisher(Float64, '/drawing/feedback/progress', 1)
        self.waypoint_index_pub = self.create_publisher(UInt32, '/drawing/feedback/waypoint_index', 1)
        
        # Control timer
        self.control_timer = None
        
        self.get_logger().info('Path follower node initialized')
    
    def state_callback(self, msg: String):
        """Handle robot state changes."""
        old_state = self.current_state
        self.current_state = msg.data
        was_active = self.active
        self.active = (self.current_state == 'drawing')
        
        if old_state != self.current_state:
            self.get_logger().info(f'State changed: {old_state} -> {self.current_state}')
        
        # Start execution when entering drawing state
        if self.active and not was_active:
            if self.stored_path is not None:
                self.get_logger().info('Entering drawing state - starting path execution')
                self.current_path = self.stored_path
                self.start_execution()
            else:
                self.get_logger().warn('Entered drawing state but no stored path available')
        
        # Stop execution when exiting drawing state
        elif not self.active and was_active:
            self.get_logger().info('Exiting drawing state - stopping execution')
            self.stop_execution()
    
    def pose_callback(self, msg: PoseStamped):
        """Update current position from odometry/localization."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        # Extract yaw from quaternion if available
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        if qz != 0.0 or qw != 0.0:
            self.current_yaw = 2.0 * math.atan2(qz, qw)
        
        if not self.has_pose:
            self.get_logger().info(f'First pose received: ({self.current_x:.3f}, {self.current_y:.3f}, yaw={self.current_yaw:.3f})')
            self.has_pose = True
    
    def imu_callback(self, msg: Imu):
        """Update current orientation from IMU."""
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.current_yaw = 2.0 * math.atan2(qz, qw)
        self.has_pose = True  # Consider IMU as pose source
    
    def path_callback(self, msg: Path):
        """Handle new path received."""
        self.stored_path = msg
        self.get_logger().info(f'Path received: {len(msg.poses)} waypoints')
        
        # Cancel current execution if new path arrives
        if self.executing:
            self.get_logger().info('New path received - canceling current execution')
            self.stop_execution()
        
        # Start execution if active
        if self.active:
            self.current_path = msg
            self.start_execution()
        else:
            self.get_logger().info('Not in drawing state - storing path for later')
    
    def cancel_callback(self, msg: Empty):
        """Handle cancellation request."""
        self.get_logger().info('Cancellation requested')
        self.cancel_requested = True
        self.stop_execution()
    
    def start_execution(self):
        """Start executing the current path."""
        if self.current_path is None or len(self.current_path.poses) == 0:
            self.get_logger().warn('No valid path to execute')
            return
        
        if not self.active:
            self.get_logger().warn('Cannot start - not in drawing state')
            return
        
        if self.executing:
            self.get_logger().warn('Execution already in progress')
            return
        
        self.get_logger().info(f'Starting path execution: {len(self.current_path.poses)} waypoints')
        
        # Reset state
        self.executing = True
        self.cancel_requested = False
        self.current_waypoint_idx = 0
        
        # Reset PID controllers
        self.prev_forward_error = 0.0
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_forward = 0.0
        self.integral_lateral = 0.0
        self.integral_heading = 0.0
        
        # Start control loop
        self.control_timer = self.create_timer(self.dt, self.control_loop)
    
    def stop_execution(self):
        """Stop current execution."""
        if not self.executing:
            return
        
        self.get_logger().info('Stopping path execution')
        
        # Cancel timer
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.control_timer = None
        
        # Stop robot
        self.publish_velocity(0.0, 0.0, 0.0)
        
        # Reset state
        self.executing = False
        self.current_waypoint_idx = 0
    
    def control_loop(self):
        """Main control loop - computes errors and publishes velocity commands."""
        if not self.executing or not self.active or self.cancel_requested:
            self.stop_execution()
            return
        
        if self.current_path is None or len(self.current_path.poses) == 0:
            self.stop_execution()
            return
        
        # Check if all waypoints completed
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('Path execution complete')
            self.stop_execution()
            return
        
        # Get current target waypoint
        target_pose = self.current_path.poses[self.current_waypoint_idx]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # Extract target heading from quaternion
        qz = target_pose.pose.orientation.z
        qw = target_pose.pose.orientation.w
        target_yaw = 2.0 * math.atan2(qz, qw) if (qz != 0.0 or qw != 0.0) else self.current_yaw
        
        # Compute errors in global frame
        dx_global = target_x - self.current_x
        dy_global = target_y - self.current_y
        distance = math.sqrt(dx_global**2 + dy_global**2)
        
        # Compute heading error (normalize to [-pi, pi])
        heading_error_global = target_yaw - self.current_yaw
        heading_error_global = self.normalize_angle(heading_error_global)
        
        # Check if waypoint reached (distance and heading thresholds)
        if distance < self.waypoint_tolerance and abs(heading_error_global) < self.heading_tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1}/{len(self.current_path.poses)} reached '
                f'(distance={distance:.3f}, heading_error={math.degrees(heading_error_global):.1f}°)'
            )
            
            # Advance to next waypoint
            self.current_waypoint_idx += 1
            
            # Reset PID controllers for new waypoint
            self.prev_forward_error = 0.0
            self.prev_lateral_error = 0.0
            self.prev_heading_error = 0.0
            self.integral_forward = 0.0
            self.integral_lateral = 0.0
            self.integral_heading = 0.0
            
            # Publish progress
            self.publish_progress()
            return
        
        # Transform position error from global frame to robot body frame
        # Body frame: x = forward, y = left (lateral)
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        # Rotate global error to body frame
        forward_error = cos_yaw * dx_global + sin_yaw * dy_global
        lateral_error = -sin_yaw * dx_global + cos_yaw * dy_global
        
        # Heading error is already in global frame, but we use it directly
        heading_error = heading_error_global
        
        # Compute desired chassis velocities using PID control
        # Forward velocity
        self.integral_forward += forward_error * self.dt
        derivative_forward = (forward_error - self.prev_forward_error) / self.dt if self.dt > 0 else 0.0
        forward_vel = (self.kp_forward * forward_error + 
                      self.ki_forward * self.integral_forward + 
                      self.kd_forward * derivative_forward)
        forward_vel = max(-self.max_forward_vel, min(self.max_forward_vel, forward_vel))
        self.prev_forward_error = forward_error
        
        # Lateral velocity
        self.integral_lateral += lateral_error * self.dt
        derivative_lateral = (lateral_error - self.prev_lateral_error) / self.dt if self.dt > 0 else 0.0
        lateral_vel = (self.kp_lateral * lateral_error + 
                      self.ki_lateral * self.integral_lateral + 
                      self.kd_lateral * derivative_lateral)
        lateral_vel = max(-self.max_lateral_vel, min(self.max_lateral_vel, lateral_vel))
        self.prev_lateral_error = lateral_error
        
        # Angular velocity
        self.integral_heading += heading_error * self.dt
        derivative_heading = (heading_error - self.prev_heading_error) / self.dt if self.dt > 0 else 0.0
        angular_vel = (self.kp_heading * heading_error + 
                      self.ki_heading * self.integral_heading + 
                      self.kd_heading * derivative_heading)
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        self.prev_heading_error = heading_error
        
        # Publish chassis velocities
        # Note: The kinematics node expects world-frame velocities in linear.x and linear.y
        # So we need to transform body-frame velocities back to world frame
        vx_world = cos_yaw * forward_vel - sin_yaw * lateral_vel
        vy_world = sin_yaw * forward_vel + cos_yaw * lateral_vel
        
        self.publish_velocity(vx_world, vy_world, angular_vel)
        self.publish_progress()
    
    def publish_velocity(self, vx: float, vy: float, omega: float):
        """Publish chassis velocity command."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
    
    def publish_progress(self):
        """Publish execution progress."""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return
        
        progress = float(self.current_waypoint_idx) / len(self.current_path.poses)
        
        progress_msg = Float64()
        progress_msg.data = progress
        self.progress_pub.publish(progress_msg)
        
        waypoint_idx_msg = UInt32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_index_pub.publish(waypoint_idx_msg)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
