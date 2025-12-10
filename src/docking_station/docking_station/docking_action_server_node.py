#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Bool, Float64
from geometry_msgs.msg import Twist, PointStamped, Point
import math
import time


class DockingActionServerNode(Node):
    def __init__(self):
        super().__init__('docking_action_server')
        
        self.declare_parameter('docking_tolerance', 0.1)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.3)
        self.declare_parameter('control_frequency', 20.0)
        
        self.tolerance = self.get_parameter('docking_tolerance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # Topic subscriptions for goal and cancel
        self.dock_trigger_sub = self.create_subscription(
            Empty,
            '/dock_robot_trigger',
            self.dock_trigger_callback,
            1
        )
        
        self.cancel_sub = self.create_subscription(
            Empty,
            '/cancel_docking',
            self.cancel_callback,
            1
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PointStamped,
            '/robot_position',
            self.position_callback,
            10
        )
        
        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/docking/cmd_vel', 1)
        
        # Feedback publishers
        self.distance_pub = self.create_publisher(Float64, '/docking/feedback/distance_to_dock', 1)
        self.current_position_pub = self.create_publisher(Point, '/docking/feedback/current_position', 1)
        
        # Result publishers
        self.success_pub = self.create_publisher(Bool, '/docking/result/success', 1)
        self.final_position_pub = self.create_publisher(Point, '/docking/result/final_position', 1)
        
        self.current_state = 'docked'
        self.current_position = None
        self.active = False
        self.executing = False
        self.cancel_requested = False
        
        self.get_logger().info('docking server initialized (topic-based)')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state == 'returning_to_docking_station')
    
    def position_callback(self, msg):
        self.current_position = msg.point
    
    def cancel_callback(self, msg):
        self.cancel_requested = True
        self.get_logger().info('docking cancellation requested')
    
    def dock_trigger_callback(self, msg):
        if self.executing:
            self.get_logger().warn('already executing docking, ignoring trigger')
            return
        
        if not self.active:
            self.get_logger().warn('not in returning_to_docking_station state, ignoring trigger')
            success_msg = Bool()
            success_msg.data = False
            self.success_pub.publish(success_msg)
            return
        
        self.executing = True
        self.cancel_requested = False
        self.execute_docking()
    
    def execute_docking(self):
        start_time = time.time()
        last_position_time = time.time()
        
        while self.active and not self.cancel_requested:
            if self.current_position is None:
                rclpy.spin_once(self, timeout_sec=1.0 / self.control_freq)
                if time.time() - last_position_time > 5.0:
                    self.get_logger().warn('no position data received, aborting docking')
                    success_msg = Bool()
                    success_msg.data = False
                    self.success_pub.publish(success_msg)
                    final_pos = Point()
                    self.final_position_pub.publish(final_pos)
                    self.executing = False
                    return
                continue
            
            last_position_time = time.time()
            
            distance = math.sqrt(
                self.current_position.x**2 + 
                self.current_position.y**2 + 
                self.current_position.z**2
            )
            
            # Publish feedback via topics
            distance_msg = Float64()
            distance_msg.data = distance
            self.distance_pub.publish(distance_msg)
            self.current_position_pub.publish(self.current_position)
            
            if distance < self.tolerance:
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                
                # Publish success result
                success_msg = Bool()
                success_msg.data = True
                self.success_pub.publish(success_msg)
                self.final_position_pub.publish(self.current_position)
                self.executing = False
                return
            
            cmd = self.compute_velocity_to_dock()
            self.cmd_vel_pub.publish(cmd)
            
            rclpy.spin_once(self, timeout_sec=1.0 / self.control_freq)
        
        # Docking aborted or cancelled
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Publish failure result
        success_msg = Bool()
        success_msg.data = False
        self.success_pub.publish(success_msg)
        
        final_pos = Point()
        if self.current_position:
            final_pos = self.current_position
        self.final_position_pub.publish(final_pos)
        
        self.executing = False
    
    def compute_velocity_to_dock(self):
        cmd = Twist()
        
        if self.current_position is None:
            return cmd
        
        target_x = -self.current_position.x
        target_y = -self.current_position.y
        
        distance = math.sqrt(target_x**2 + target_y**2)
        angle = math.atan2(target_y, target_x)
        
        if distance < self.tolerance:
            return cmd
        
        linear_vel = min(self.max_linear, distance * 1.5)
        angular_vel = max(-self.max_angular, min(self.max_angular, angle * 1.5))
        
        cmd.linear.x = linear_vel * math.cos(angle)
        cmd.linear.y = linear_vel * math.sin(angle)
        cmd.angular.z = angular_vel
        
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = DockingActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

