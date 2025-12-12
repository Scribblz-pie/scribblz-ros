#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt32
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
import math
from typing import Optional


class DrawingControlNode(Node):
    def __init__(self):
        super().__init__('drawing_control')
        
        # Parameters
        self.declare_parameter('waypoint_tolerance', 0.05)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('kp', 2.0)  # Proportional gain
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('obstacle_threshold', 0.15)  # meters
        
        self.tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 1)
        self.waypoint_index_pub = self.create_publisher(UInt32, '/drawing/feedback/waypoint_index', 1)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/execute_drawing_path',
            self.path_callback,
            1
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
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
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.current_orientation = 0.0
        self.lidar_scan = None
        self.current_state = 'docked'
        self.active = False
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.control_freq, self.control_loop)
        
        self.get_logger().info('Drawing control node initialized')
    
    def state_callback(self, msg: String):
        """Handle robot state changes."""
        self.current_state = msg.data
        self.active = (self.current_state == 'drawing')
        if not self.active:
            # Stop robot when not in drawing state
            self.publish_stop()
            self.current_waypoint_idx = 0
    
    def path_callback(self, msg: Path):
        """Handle new path from waypoint publisher."""
        if not self.active:
            self.get_logger().warn('Received path but not in drawing state')
            return
        
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
    
    def lidar_callback(self, msg: LaserScan):
        """Store latest lidar scan."""
        self.lidar_scan = msg
    
    def imu_callback(self, msg: Imu):
        """Extract orientation from IMU."""
        # Convert quaternion to yaw
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.current_orientation = 2.0 * math.atan2(qz, qw)
    
    def pose_callback(self, msg: PoseStamped):
        """Store current robot pose."""
        self.current_pose = msg.pose
    
    def control_loop(self):
        """Main control loop - runs at control_frequency."""
        if not self.active or self.current_path is None:
            return
        
        if self.current_waypoint_idx >= len(self.current_path.poses):
            # Path complete
            self.publish_stop()
            return
        
        if self.current_pose is None:
            # No pose data yet
            return
        
        # Get target waypoint
        target_pose = self.current_path.poses[self.current_waypoint_idx].pose
        
        # Check if we've reached current waypoint
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)
        
        if distance < self.tolerance:
            # Reached waypoint, move to next
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path.poses):
                self.publish_stop()
                return
        
        # Compute velocity command
        cmd = self.compute_velocity_command(dx, dy, distance)
        
        # Apply obstacle avoidance
        cmd = self.apply_obstacle_avoidance(cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish feedback
        idx_msg = UInt32()
        idx_msg.data = self.current_waypoint_idx
        self.waypoint_index_pub.publish(idx_msg)
    
    def compute_velocity_command(self, dx: float, dy: float, distance: float) -> Twist:
        """Compute velocity command using proportional control."""
        cmd = Twist()
        
        if distance < self.tolerance:
            return cmd
        
        # Proportional control: v = kp * error
        # Error is the distance to waypoint
        desired_vel = self.kp * distance
        
        # Limit velocity
        desired_vel = min(desired_vel, self.max_linear)
        
        # Direction to waypoint
        angle = math.atan2(dy, dx)
        
        # Set velocity components
        cmd.linear.x = desired_vel * math.cos(angle)
        cmd.linear.y = desired_vel * math.sin(angle)
        
        return cmd
    
    def apply_obstacle_avoidance(self, cmd: Twist) -> Twist:
        """Reduce velocity if obstacles detected."""
        if self.lidar_scan is None:
            return cmd
        
        # Check for obstacles in the direction of travel
        cmd_angle = math.atan2(cmd.linear.y, cmd.linear.x)
        cmd_magnitude = math.hypot(cmd.linear.x, cmd.linear.y)
        
        # Find minimum distance in direction of travel
        min_distance = float('inf')
        
        if len(self.lidar_scan.ranges) > 0:
            angle_min = self.lidar_scan.angle_min
            angle_increment = self.lidar_scan.angle_increment
            
            for i, range_val in enumerate(self.lidar_scan.ranges):
                if math.isnan(range_val) or math.isinf(range_val):
                    continue
                
                scan_angle = angle_min + i * angle_increment
                # Check if obstacle is in direction of travel (within ±45 degrees)
                angle_diff = abs(math.atan2(math.sin(scan_angle - cmd_angle), 
                                           math.cos(scan_angle - cmd_angle)))
                
                if angle_diff < math.pi / 4.0:  # 45 degrees
                    if range_val < min_distance:
                        min_distance = range_val
        
        # Reduce velocity if obstacle too close
        if min_distance < self.obstacle_threshold:
            reduction_factor = min_distance / self.obstacle_threshold
            cmd.linear.x *= reduction_factor
            cmd.linear.y *= reduction_factor
            self.get_logger().warn(f'Obstacle detected at {min_distance:.3f}m, reducing velocity')
        
        return cmd
    
    def publish_stop(self):
        """Publish zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DrawingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

