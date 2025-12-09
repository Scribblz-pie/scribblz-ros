#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, Point
from docking_station.action import DockRobot
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
        
        self.action_server = ActionServer(
            self,
            DockRobot,
            '/dock_robot',
            self.execute_callback
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
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/docking/cmd_vel', 1)
        
        self.current_state = 'docked'
        self.current_position = None
        self.active = False
        
        self.get_logger().info('docking action server initialized')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state == 'returning_to_docking_station')
    
    def position_callback(self, msg):
        self.current_position = msg.point
    
    def execute_callback(self, goal_handle):
        if not self.active:
            goal_handle.abort()
            result = DockRobot.Result()
            result.success = False
            result.final_position = Point()
            return result
        
        start_time = time.time()
        last_position_time = time.time()
        
        while self.active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DockRobot.Result()
                result.success = False
                result.final_position = Point()
                if self.current_position:
                    result.final_position = self.current_position
                return result
            
            if self.current_position is None:
                rclpy.spin_once(self, timeout_sec=1.0 / self.control_freq)
                if time.time() - last_position_time > 5.0:
                    self.get_logger().warn('no position data received, aborting docking')
                    goal_handle.abort()
                    result = DockRobot.Result()
                    result.success = False
                    result.final_position = Point()
                    return result
                continue
            
            last_position_time = time.time()
            
            distance = math.sqrt(
                self.current_position.x**2 + 
                self.current_position.y**2 + 
                self.current_position.z**2
            )
            
            if distance < self.tolerance:
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                
                goal_handle.succeed()
                result = DockRobot.Result()
                result.success = True
                result.final_position = self.current_position
                return result
            
            cmd = self.compute_velocity_to_dock()
            self.cmd_vel_pub.publish(cmd)
            
            feedback = DockRobot.Feedback()
            feedback.distance_to_dock = distance
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)
            
            rclpy.spin_once(self, timeout_sec=1.0 / self.control_freq)
        
        goal_handle.abort()
        result = DockRobot.Result()
        result.success = False
        result.final_position = Point()
        if self.current_position:
            result.final_position = self.current_position
        return result
    
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

