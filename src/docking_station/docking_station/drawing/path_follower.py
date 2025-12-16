#!/usr/bin/env python3
"""
Path follower node for kiwi drive robot.

Implements high-level path following by:
1. Using TF2 to transform waypoints from map frame to robot body frame (base_link)
2. Computing position and heading errors in robot body frame
3. Using PID control to compute desired chassis velocities in body frame
4. Publishing body-frame velocities directly (kinematics node converts to wheel velocities)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Bool, UInt32, Float64
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
import math
from typing import Optional
import tf2_ros
from tf2_ros import TransformException, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation




class PathFollowerNode(Node):
    """Path follower that converts waypoint errors to chassis velocity commands."""
    
    def __init__(self):
        super().__init__('path_follower')
        
        # Declare parameters
        self.declare_parameter('waypoint_tolerance', 0.05)  # Distance threshold (m)
        self.declare_parameter('heading_tolerance', 0.1)  # Heading threshold (rad)
        self.declare_parameter('max_velocity', 1.0)  # Max translation velocity (m/s) - matches teleop
        self.declare_parameter('max_angular_velocity', 3.0)  # Max angular velocity (rad/s) - matches teleop
        self.declare_parameter('control_frequency', 20.0)  # Control loop frequency (Hz)
        
        # Simplified proportional control parameters
        self.declare_parameter('control_gain', 0.6)  # Overall control aggressiveness (0-1)
        
        # Characteristic scales - defines error magnitude for full speed
        self.declare_parameter('forward_scale', 0.5)  # meters - full speed at 0.5m forward error
        self.declare_parameter('lateral_scale', 0.5)  # meters - full speed at 0.5m lateral error
        self.declare_parameter('heading_scale', 1.0)  # radians - full speed at 1 radian (~57°) heading error
        
        # Dead zone parameters - stop sending commands when very close to target
        self.declare_parameter('position_dead_zone', 0.03)  # meters - no movement within 3cm
        self.declare_parameter('heading_dead_zone', 0.08)  # radians - no rotation within ~4.6°
        
        # Get parameter values
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.heading_tolerance = self.get_parameter('heading_tolerance').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        self.control_gain = self.get_parameter('control_gain').get_parameter_value().double_value
        self.forward_scale = self.get_parameter('forward_scale').get_parameter_value().double_value
        self.lateral_scale = self.get_parameter('lateral_scale').get_parameter_value().double_value
        self.heading_scale = self.get_parameter('heading_scale').get_parameter_value().double_value
        
        self.position_dead_zone = self.get_parameter('position_dead_zone').get_parameter_value().double_value
        self.heading_dead_zone = self.get_parameter('heading_dead_zone').get_parameter_value().double_value
        
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
        
        # Control state
        self.dt = 1.0 / self.control_freq
        
        # Setup subscribers
        self.path_sub = self.create_subscription(
            Path, '/execute_drawing_path', self.path_callback, 1)
        
        self.cancel_sub = self.create_subscription(
            Empty, '/cancel_drawing', self.cancel_callback, 1)
        
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        
        # Setup publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 1)
        self.progress_pub = self.create_publisher(Float64, '/drawing/feedback/progress', 1)
        self.waypoint_index_pub = self.create_publisher(UInt32, '/drawing/feedback/waypoint_index', 1)
        
        # Setup TF2 transform listener and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Control timer
        self.control_timer = None
        
        # Status logging timer (runs continuously regardless of state)
        self.create_timer(1.0, self.status_logging_callback)  # 1 Hz status updates
        
        self.get_logger().info('Path follower node initialized with TF2 support')
    
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
        """Update current position from odometry/localization.
        
        Pose is received in map frame. We'll transform it to base_link frame
        when needed using TF2. Also publish the transform from map to base_link.
        """
        # Store pose in map frame (will transform to base_link when needed)
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        # Extract yaw from quaternion using proper conversion
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.current_yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
        
        # Publish transform from map to base_link
        # Orientation is already corrected in /robot_pose by lidar_pose node
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        
        if not self.has_pose:
            self.get_logger().info(
                f'First pose received: pos=({self.current_x:.3f}, {self.current_y:.3f}), '
                f'yaw={math.degrees(self.current_yaw):.1f}°'
            )
            self.has_pose = True
    
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
    
    def status_logging_callback(self):
        """Continuous status logging (runs regardless of state)."""
        if not self.has_pose:
            return
        
        # Always log current pose
        self.get_logger().debug(
            f'[STATUS] State: {self.current_state} | Pose: ({self.current_x:.3f}, {self.current_y:.3f}), yaw={math.degrees(self.current_yaw):.1f}°'
        )
        
        # If we have a stored path, log errors to the current/next waypoint
        if self.stored_path is not None and len(self.stored_path.poses) > 0:
            target_idx = self.current_waypoint_idx if self.executing else 0
            if target_idx < len(self.stored_path.poses):
                target_pose = self.stored_path.poses[target_idx]
                
                # Transform waypoint from map frame to base_link frame using TF2
                waypoint_in_base_link = self.transform_waypoint_to_base_link(target_pose)
                
                # Calculate errors in map frame for logging
                x_error = target_pose.pose.position.x - self.current_x
                y_error = target_pose.pose.position.y - self.current_y
                
                # Fallback to manual transformation if TF2 fails
                if waypoint_in_base_link is None:
                    # Extract target yaw
                    target_q = target_pose.pose.orientation
                    target_yaw = 2.0 * math.atan2(target_q.z, target_q.w) if (target_q.z != 0.0 or target_q.w != 0.0) else self.current_yaw
                    # Use atan2 method to always pick shortest rotation path and avoid ±180° discontinuity
                    angle_diff = target_yaw - self.current_yaw
                    heading_error = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    
                    # Transform to robot frame manually
                    cos_yaw = math.cos(self.current_yaw)
                    sin_yaw = math.sin(self.current_yaw)
                    forward_error = cos_yaw * x_error + sin_yaw * y_error
                    lateral_error = -sin_yaw * x_error + cos_yaw * y_error
                else:
                    # TF2 transformation successful - errors are the transformed coordinates
                    forward_error = waypoint_in_base_link[0]
                    lateral_error = waypoint_in_base_link[1]
                    heading_error = waypoint_in_base_link[2]
                
                self.get_logger().debug(
                    f'[STATUS] WP[{target_idx}]: target=({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f})'
                )
                self.get_logger().debug(
                    f'[STATUS] MAP ERRORS: x={x_error:.3f}m, y={y_error:.3f}m'
                )
                self.get_logger().debug(
                    f'[STATUS] BASE_LINK ERRORS (TF2): fwd={forward_error:.3f}m, lat={lateral_error:.3f}m, hdg={math.degrees(heading_error):.1f}°'
                )
    
    def get_robot_pose_in_base_link(self) -> Optional[tuple]:
        """Get current robot pose in base_link frame using TF2.
        
        Returns:
            Tuple of (x, y, yaw) in base_link frame, or None if transform unavailable
        """
        try:
            # Try to get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'map',  # source frame
                rclpy.time.Time()
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
            
            return (x, y, yaw)
        except TransformException as ex:
            # If transform not available, try using current pose and IMU yaw
            # This is a fallback for when TF2 transform isn't set up yet
            if self.has_pose:
                # Use stored pose (in map frame) - will need manual transform
                # For now, return None to indicate we should use fallback
                return None
            return None
    
    def transform_waypoint_to_base_link(self, waypoint_pose: PoseStamped) -> Optional[tuple]:
        """Transform waypoint from map frame to base_link frame using TF2.
        
        Args:
            waypoint_pose: PoseStamped in map frame
            
        Returns:
            Tuple of (x, y, yaw) in base_link frame, or None if transform fails
            
        Verification examples:
        - Robot at (0,0), yaw=0°, waypoint at (1,0) → base_link: (1,0,0) [1m forward]
        - Robot at (0,0), yaw=90°, waypoint at (0,1) → base_link: (1,0,0) [1m forward]
        - Robot at (0,0), yaw=180°, waypoint at (1,0) → base_link: (-1,0,0) [1m behind]
        - Robot at (1,1), yaw=0°, waypoint at (2,1.5) → base_link: (1,0.5,0) [1m forward, 0.5m left]
        """
        try:
            # Use TF2's built-in transformation - it handles all the rotation math for us!
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                waypoint_pose.header.frame_id,  # source frame (should be 'map')
                rclpy.time.Time()
            )
            
            # Transform the pose using TF2's built-in utility
            # This automatically handles translation and rotation transformation
            transformed_pose = do_transform_pose(waypoint_pose.pose, transform)
            
            # Extract transformed position (waypoint in base_link frame)
            wp_x_base = transformed_pose.position.x
            wp_y_base = transformed_pose.position.y
            
            # Extract transformed orientation
            wp_q = transformed_pose.orientation
            wp_yaw_base = Rotation.from_quat([wp_q.x, wp_q.y, wp_q.z, wp_q.w]).as_euler('xyz')[2]
            
            return (wp_x_base, wp_y_base, wp_yaw_base)
        except TransformException as ex:
            self.get_logger().warn(f'Failed to transform waypoint to base_link: {ex}')
            return None
    
    def control_loop(self):
        """Main control loop - computes errors in body frame and publishes velocity commands."""
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
        
        # Get target waypoint
        target_pose = self.current_path.poses[self.current_waypoint_idx]
        
        # Transform waypoint from map frame to base_link frame using TF2
        # Key concept: In base_link frame, robot is at origin (0,0,0), so:
        #   - forward_error = waypoint.x (distance along robot's forward axis)
        #   - lateral_error = waypoint.y (distance along robot's lateral axis)
        #   - heading_error = waypoint.yaw (angular difference)
        waypoint_in_base_link = self.transform_waypoint_to_base_link(target_pose)
        
        # Fallback to manual transformation if TF2 fails
        if waypoint_in_base_link is None:
            self.get_logger().warn('TF2 transform failed - using fallback manual transformation')
            
            # Calculate errors in map frame
            x_error = target_pose.pose.position.x - self.current_x
            y_error = target_pose.pose.position.y - self.current_y
            
            # Extract target yaw from quaternion
            target_q = target_pose.pose.orientation
            target_yaw = 2.0 * math.atan2(target_q.z, target_q.w) if (target_q.z != 0.0 or target_q.w != 0.0) else self.current_yaw
            # Use atan2 method to always pick shortest rotation path and avoid ±180° discontinuity
            angle_diff = target_yaw - self.current_yaw
            heading_error = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Calculate distance for waypoint completion check
            distance = math.sqrt(x_error**2 + y_error**2)
            
            # Transform map frame errors to robot body frame using rotation
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            
            # Rotate error vector from map frame to robot body frame
            forward_error = cos_yaw * x_error + sin_yaw * y_error
            lateral_error = -sin_yaw * x_error + cos_yaw * y_error
            
            self.get_logger().debug(
                f'[WP {self.current_waypoint_idx}] FALLBACK: MAP ERRORS: x={x_error:.3f}m, y={y_error:.3f}m'
            )
        else:
            # TF2 transformation successful - errors are the transformed coordinates
            # Since robot is at origin (0,0,0) in base_link frame:
            # - forward_error = waypoint.x in base_link
            # - lateral_error = waypoint.y in base_link
            # - heading_error = waypoint.yaw in base_link
            forward_error = waypoint_in_base_link[0]
            lateral_error = waypoint_in_base_link[1]
            heading_error = waypoint_in_base_link[2]
            
            # Calculate distance for waypoint completion check
            distance = math.sqrt(forward_error**2 + lateral_error**2)
            
            # Calculate map frame errors for logging
            x_error = target_pose.pose.position.x - self.current_x
            y_error = target_pose.pose.position.y - self.current_y
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance and abs(heading_error) < self.heading_tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1}/{len(self.current_path.poses)} reached '
                f'(distance={distance:.3f}m, heading_error={math.degrees(heading_error):.1f}°)'
            )
            self.current_waypoint_idx += 1
            self.publish_progress()
            return
        
        # Debug: Log errors in both frames
        self.get_logger().debug(
            f'[WP {self.current_waypoint_idx}] MAP FRAME: target=({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}) '
            f'robot=({self.current_x:.3f}, {self.current_y:.3f}, yaw={math.degrees(self.current_yaw):.1f}°)'
        )
        self.get_logger().debug(
            f'[WP {self.current_waypoint_idx}] MAP ERRORS: x_error={x_error:.3f}m, y_error={y_error:.3f}m'
        )
        self.get_logger().debug(
            f'[WP {self.current_waypoint_idx}] BASE_LINK ERRORS (TF2): forward={forward_error:.3f}m, lateral={lateral_error:.3f}m, heading={math.degrees(heading_error):.1f}°'
        )
        
        # Apply dead zone - zero out errors that are too small to prevent oscillation
        forward_error_original = forward_error
        lateral_error_original = lateral_error
        heading_error_original = heading_error
        
        if abs(forward_error) < self.position_dead_zone:
            forward_error = 0.0
        if abs(lateral_error) < self.position_dead_zone:
            lateral_error = 0.0
        if abs(heading_error) < self.heading_dead_zone:
            heading_error = 0.0
        
        # If all errors are within dead zone, stop completely
        if forward_error == 0.0 and lateral_error == 0.0 and heading_error == 0.0:
            self.get_logger().debug(
                f'[CONTROL] All errors in dead zone - stopping robot '
                f'(fwd={forward_error_original:.4f}m, lat={lateral_error_original:.4f}m, hdg={math.degrees(heading_error_original):.2f}°)'
            )
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        
        # Compute desired chassis velocities using normalized proportional control
        # STEP 1: Normalize errors to [-1, 1] based on characteristic scales
        forward_norm = max(-1.0, min(1.0, forward_error / self.forward_scale))
        lateral_norm = max(-1.0, min(1.0, lateral_error / self.lateral_scale))
        heading_norm = max(-1.0, min(1.0, heading_error / self.heading_scale))
        
        # STEP 2: Apply proportional gain to get desired velocities
        forward_vel = forward_norm * self.control_gain * self.max_velocity
        lateral_vel = lateral_norm * self.control_gain * self.max_velocity
        angular_vel = heading_norm * self.control_gain * self.max_angular_velocity
        
        # STEP 3: Proportional saturation - scale all velocities together if any would exceed max
        # This preserves the direction vector to the waypoint
        translation_magnitude = math.sqrt(forward_vel**2 + lateral_vel**2)
        
        # Normalize angular to same scale for comparison
        angular_magnitude_normalized = abs(angular_vel) / self.max_angular_velocity * self.max_velocity
        
        # Find the maximum magnitude
        max_magnitude = max(translation_magnitude, angular_magnitude_normalized)
        
        # If any velocity would exceed max, scale all proportionally
        if max_magnitude > self.max_velocity:
            scale_factor = self.max_velocity / max_magnitude
            forward_vel *= scale_factor
            lateral_vel *= scale_factor
            angular_vel *= scale_factor
            self.get_logger().debug(f'[CONTROL] Applying proportional saturation: scale_factor={scale_factor:.3f}')
        
        # Log normalized errors and control outputs
        self.get_logger().debug(
            f'[CONTROL] Normalized errors: fwd={forward_norm:.3f}, lat={lateral_norm:.3f}, hdg={heading_norm:.3f}'
        )
        
        # Debug logging
        self.get_logger().debug(
            f'[CONTROL] ROBOT FRAME -> Velocities: vx(forward)={forward_vel:.3f}m/s, vy(lateral)={lateral_vel:.3f}m/s, wz(angular)={angular_vel:.3f}rad/s'
        )
        
        # Publish body-frame velocities directly (like teleop)
        # vx = forward (robot +x), vy = left (robot +y), wz = rotation
        self.publish_velocity(forward_vel, lateral_vel, angular_vel)
        self.publish_progress()
    
    def publish_velocity(self, vx: float, vy: float, omega: float):
        """Publish chassis velocity command in body frame.
        
        Args:
            vx: Forward velocity (m/s) - positive = forward
            vy: Lateral velocity (m/s) - positive = left
            omega: Angular velocity (rad/s) - positive = counterclockwise
        """
        cmd = Twist()
        cmd.linear.x = float(vx)  # Forward
        cmd.linear.y = float(vy)  # Lateral (left)
        cmd.angular.z = float(omega)  # Angular
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
        """Normalize angle to [-pi, pi].
        
        Note: For computing angle errors, prefer using atan2(sin(diff), cos(diff))
        instead, as it avoids discontinuity issues at ±180° and always picks
        the shortest rotation path.
        """
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
