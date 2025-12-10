#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Empty, Bool, UInt32, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
import time


class DrawingActionServerNode(Node):
    def __init__(self):
        super().__init__('drawing_action_server')
        
        self.declare_parameter('waypoint_tolerance', 0.05)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('target_fan_speed', 255)
        self.declare_parameter('fan_ramp_steps', 10)
        self.declare_parameter('fan_ramp_delay', 0.1)
        self.declare_parameter('imu_reset_topic', '/imu/reset')
        self.declare_parameter('initialization_timeout', 5.0)
        
        self.tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.target_fan_speed = self.get_parameter('target_fan_speed').get_parameter_value().integer_value
        self.fan_ramp_steps = self.get_parameter('fan_ramp_steps').get_parameter_value().integer_value
        self.fan_ramp_delay = self.get_parameter('fan_ramp_delay').get_parameter_value().double_value
        self.imu_reset_topic = self.get_parameter('imu_reset_topic').get_parameter_value().string_value
        self.init_timeout = self.get_parameter('initialization_timeout').get_parameter_value().double_value
        
        # Topic subscriptions for goal and cancel
        self.path_sub = self.create_subscription(
            Path,
            '/execute_drawing_path',
            self.path_callback,
            1
        )
        
        self.cancel_sub = self.create_subscription(
            Empty,
            '/cancel_drawing',
            self.cancel_callback,
            1
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )
        
        # Command publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 1)
        self.fan_pub = self.create_publisher(Int32, '/fan_speed', 1)
        self.imu_reset_pub = self.create_publisher(Empty, self.imu_reset_topic, 1)
        
        # Feedback publishers
        self.progress_pub = self.create_publisher(Float64, '/drawing/feedback/progress', 1)
        self.waypoint_index_pub = self.create_publisher(UInt32, '/drawing/feedback/waypoint_index', 1)
        
        # Result publishers
        self.success_pub = self.create_publisher(Bool, '/drawing/result/success', 1)
        self.completion_time_pub = self.create_publisher(Float64, '/drawing/result/completion_time', 1)
        
        self.current_state = 'docked'
        self.active = False
        self.current_path = None
        self.executing = False
        self.cancel_requested = False
        
        self.get_logger().info('drawing server initialized (topic-based)')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state == 'drawing')
    
    def cancel_callback(self, msg):
        self.cancel_requested = True
        self.get_logger().info('drawing cancellation requested')
    
    def path_callback(self, msg):
        if self.executing:
            self.get_logger().warn('already executing a path, ignoring new path')
            return
        
        if not self.active:
            self.get_logger().warn('not in drawing state, ignoring path')
            success_msg = Bool()
            success_msg.data = False
            self.success_pub.publish(success_msg)
            return
        
        self.current_path = msg
        self.executing = True
        self.cancel_requested = False
        
        # Start execution
        self.get_logger().info(f'received path with {len(msg.poses)} waypoints')
        self.execute_path()
    
    def initialize_for_drawing(self):
        self.get_logger().info('initializing for drawing: resetting imu and ramping fan')
        
        # reset imu
        reset_msg = Empty()
        self.imu_reset_pub.publish(reset_msg)
        self.get_logger().info('imu reset command sent')
        
        # ramp fan gradually
        if self.fan_ramp_steps > 0:
            step_size = max(1, self.target_fan_speed // self.fan_ramp_steps)
            for speed in range(0, self.target_fan_speed + 1, step_size):
                if self.cancel_requested:
                    return False
                
                fan_msg = Int32()
                fan_msg.data = speed
                self.fan_pub.publish(fan_msg)
                rclpy.spin_once(self, timeout_sec=self.fan_ramp_delay)
        else:
            fan_msg = Int32()
            fan_msg.data = self.target_fan_speed
            self.fan_pub.publish(fan_msg)
        
        # wait for fan to stabilize
        rclpy.spin_once(self, timeout_sec=0.5)
        
        self.get_logger().info('initialization complete')
        return True
    
    def execute_path(self):
        if self.current_path is None or len(self.current_path.poses) == 0:
            self.get_logger().warn('no valid path to execute')
            success_msg = Bool()
            success_msg.data = False
            self.success_pub.publish(success_msg)
            self.executing = False
            return
        
        # initialize before drawing
        if not self.initialize_for_drawing():
            success_msg = Bool()
            success_msg.data = False
            self.success_pub.publish(success_msg)
            self.executing = False
            return
        
        start_time = time.time()
        current_waypoint_idx = 0
        current_x = 0.0
        current_y = 0.0
        dt = 1.0 / self.control_freq
        
        waypoints = self.current_path.poses
        
        while current_waypoint_idx < len(waypoints) and self.active and not self.cancel_requested:
            target_pose = waypoints[current_waypoint_idx]
            target = target_pose.pose.position
            
            dx = target.x - current_x
            dy = target.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.tolerance:
                current_waypoint_idx += 1
                if current_waypoint_idx >= len(waypoints):
                    break
                continue
            
            cmd = self.compute_velocity_to_waypoint(dx, dy)
            self.cmd_vel_pub.publish(cmd)
            
            current_x += cmd.linear.x * dt
            current_y += cmd.linear.y * dt
            
            # Publish feedback via topics
            progress_msg = Float64()
            progress_msg.data = float(current_waypoint_idx) / len(waypoints)
            self.progress_pub.publish(progress_msg)
            
            waypoint_idx_msg = UInt32()
            waypoint_idx_msg.data = current_waypoint_idx
            self.waypoint_index_pub.publish(waypoint_idx_msg)
            
            rclpy.spin_once(self, timeout_sec=dt)
        
        completion_time = time.time() - start_time
        
        # Publish result
        if not self.active or self.cancel_requested:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            success_msg = Bool()
            success_msg.data = False
            self.success_pub.publish(success_msg)
        else:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            success_msg = Bool()
            success_msg.data = True
            self.success_pub.publish(success_msg)
        
        time_msg = Float64()
        time_msg.data = completion_time
        self.completion_time_pub.publish(time_msg)
        
        self.executing = False
        self.current_path = None
    
    def compute_velocity_to_waypoint(self, dx, dy):
        cmd = Twist()
        
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.tolerance:
            return cmd
        
        angle = math.atan2(dy, dx)
        linear_vel = min(self.max_linear, distance * 2.0)
        
        cmd.linear.x = linear_vel * math.cos(angle)
        cmd.linear.y = linear_vel * math.sin(angle)
        cmd.angular.z = 0.0
        
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = DrawingActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

