#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = 'docked'
        self.valid_states = ['docked', 'drawing', 'returning_to_docking_station', 'teleop', 'error', 'recovering']
        
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Service to set robot state - using SetBool as a simple trigger
        # Note: This is a simplified replacement. For full state management,
        # you may want to use a topic-based approach instead.
        # Consider subscribing to '/set_robot_state_topic' (String) for proper state management
        self.set_state_srv = self.create_service(
            SetBool,
            '/set_robot_state',
            self.set_state_callback
        )
        
        # Alternative: topic-based state management (recommended)
        self.set_state_sub = self.create_subscription(
            String,
            '/set_robot_state_topic',
            self.set_state_topic_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            Point,
            '/battery_status',
            self.battery_callback,
            10
        )
        
        self.drawing_cmd_sub = self.create_subscription(
            Twist,
            '/drawing/cmd_vel',
            self.drawing_cmd_callback,
            1
        )
        
        self.docking_cmd_sub = self.create_subscription(
            Twist,
            '/docking/cmd_vel',
            self.docking_cmd_callback,
            1
        )
        
        self.teleop_cmd_sub = self.create_subscription(
            Twist,
            '/teleop/cmd_vel',
            self.teleop_cmd_callback,
            1
        )
        
        # Topic publishers to trigger drawing and docking
        self.dock_trigger_pub = self.create_publisher(Empty, '/dock_robot_trigger', 1)
        
        self.current_cmd = None
        self.low_battery = False
        
        # Subscribe to /robot_state to accept manual state changes
        # We'll ignore messages that match our current state (likely our own publishes)
        # and only process messages that are different (external changes)
        self.robot_state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('state machine initialized, starting in docked state')
    
    def set_state_callback(self, request, response):
        # Simplified: SetBool.data=True triggers state transition
        # Note: SetBool only has 'success' field, no 'message' field
        # For full state management, use the topic-based approach via '/set_robot_state_topic'
        response.success = True
        return response
    
    def set_state_topic_callback(self, msg):
        """Topic-based state management - recommended approach"""
        new_state = msg.data
        if new_state in self.valid_states:
            if self.is_valid_transition(self.state, new_state):
                self.state = new_state
                self.get_logger().info(f'state changed to: {new_state}')
                
                if new_state == 'drawing':
                    # Note: Drawing requires a path to be published to /execute_drawing_path
                    # The state machine doesn't publish the path - external node should do this
                    pass
                elif new_state == 'returning_to_docking_station':
                    self.start_docking()
            else:
                self.get_logger().warn(f'invalid transition from {self.state} to {new_state}')
        else:
            self.get_logger().warn(f'invalid state: {new_state}')
    
    def robot_state_callback(self, msg):
        """Handle manual state changes from /robot_state topic"""
        new_state = msg.data
        # Ignore messages that match our current state (likely our own publishes)
        # Only process if it's different and valid
        if new_state == self.state:
            return  # This is likely our own message, ignore it
        
        if new_state in self.valid_states:
            if self.is_valid_transition(self.state, new_state):
                self.state = new_state
                self.get_logger().info(f'state changed to: {new_state} (from external /robot_state message)')
                
                if new_state == 'drawing':
                    pass
                elif new_state == 'returning_to_docking_station':
                    self.start_docking()
            else:
                self.get_logger().warn(f'invalid transition from {self.state} to {new_state} (from external message)')
    
    def is_valid_transition(self, from_state, to_state):
        if to_state == 'teleop' or to_state == 'error':
            return True
        if from_state == 'teleop' and to_state == 'docked':
            return True
        if from_state == 'error' and to_state == 'recovering':
            return True
        if from_state == 'recovering' and to_state == 'docked':
            return True
        if from_state == 'docked' and to_state == 'drawing':
            return True
        if from_state == 'drawing' and to_state == 'returning_to_docking_station':
            return True
        if from_state == 'returning_to_docking_station' and to_state == 'docked':
            return True
        return False
    
    def battery_callback(self, msg):
        # Point message: x=voltage, y=level, z=low_battery (0.0 or 1.0)
        self.low_battery = (msg.z > 0.5)
        if self.low_battery and self.state == 'drawing':
            self.get_logger().warn('low battery detected, transitioning to returning_to_docking_station')
            self.state = 'returning_to_docking_station'
            self.start_docking()
    
    def drawing_cmd_callback(self, msg):
        if self.state == 'drawing':
            self.current_cmd = msg
    
    def docking_cmd_callback(self, msg):
        if self.state == 'returning_to_docking_station':
            self.current_cmd = msg
    
    def teleop_cmd_callback(self, msg):
        if self.state == 'teleop':
            self.current_cmd = msg
    
    def start_docking(self):
        """Trigger docking by publishing to topic"""
        trigger_msg = Empty()
        self.dock_trigger_pub.publish(trigger_msg)
        self.get_logger().info('docking trigger sent')
    
    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
    
    def timer_callback(self):
        if self.state == 'docked' or self.state == 'error':
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.current_cmd = None
        elif self.current_cmd is not None:
            self.cmd_vel_pub.publish(self.current_cmd)
        else:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

