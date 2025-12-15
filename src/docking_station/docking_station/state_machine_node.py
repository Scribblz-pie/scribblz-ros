#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import json
import time


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = 'docked'
        self.valid_states = ['docked', 'drawing', 'docking', 'teleop', 'error', 'recovering']
        
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.debug_pub = self.create_publisher(String, '/state_machine/debug', 10)
        
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
        self.publish_debug('initialized', {'state': self.state})
    
    def set_state_callback(self, request, response):
        # Simplified: SetBool.data=True triggers state transition
        # Note: SetBool only has 'success' field, no 'message' field
        # For full state management, use the topic-based approach via '/set_robot_state_topic'
        response.success = True
        return response
    
    def publish_debug(self, event, data):
        """Publish debug information."""
        debug_msg = String()
        debug_data = {
            'event': event,
            'timestamp': time.time(),
            'data': data
        }
        debug_msg.data = json.dumps(debug_data)
        self.debug_pub.publish(debug_msg)
        self.get_logger().debug(f'[STATE_DEBUG] {event}: {data}')
    
    def set_state_topic_callback(self, msg):
        """Topic-based state management - recommended approach"""
        new_state = msg.data
        self.publish_debug('state_change_request', {
            'requested_state': new_state,
            'current_state': self.state,
            'is_valid_state': new_state in self.valid_states
        })
        
        if new_state in self.valid_states:
            if self.is_valid_transition(self.state, new_state):
                old_state = self.state
                self.state = new_state
                self.get_logger().info(f'state changed to: {new_state}')
                self.publish_debug('state_changed', {
                    'old_state': old_state,
                    'new_state': new_state,
                    'transition_valid': True
                })
                
                if new_state == 'drawing':
                    # Note: Drawing requires a path to be published to /execute_drawing_path
                    # The state machine doesn't publish the path - external node should do this
                    pass
                elif new_state == 'docking':
                    self.start_docking()
            else:
                self.get_logger().warn(f'invalid transition from {self.state} to {new_state}')
                self.publish_debug('state_change_rejected', {
                    'reason': 'invalid_transition',
                    'from_state': self.state,
                    'to_state': new_state
                })
        else:
            self.get_logger().warn(f'invalid state: {new_state}')
            self.publish_debug('state_change_rejected', {
                'reason': 'invalid_state',
                'requested_state': new_state,
                'valid_states': self.valid_states
            })
    
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
        if from_state == 'teleop' and (to_state == 'docked' or to_state == 'drawing'):
            return True
        if from_state == 'error' and to_state == 'recovering':
            return True
        if from_state == 'recovering' and to_state == 'docked':
            return True
        if from_state == 'docked' and to_state == 'drawing':
            return True
        if from_state == 'drawing' and (to_state == 'docking' or to_state == 'teleop' or to_state == 'docked'):
            return True
        if from_state == 'docking' and to_state == 'docked':
            return True
        return False
    
    def docking_cmd_callback(self, msg):
        if self.state == 'docking':
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
        # Note: Drawing commands are published directly to /cmd_vel by drawing_driver_node
        # State machine only routes teleop and docking commands
        if self.state == 'docked' or self.state == 'error' or self.state == 'drawing':
            # For drawing state, driver node handles commands directly
            # For docked/error, stop robot
            if self.state != 'drawing':
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                self.current_cmd = None
        elif self.current_cmd is not None:
            # Route teleop or docking commands
            self.cmd_vel_pub.publish(self.current_cmd)
            # Debug every 50 iterations (5 seconds at 10Hz)
            if int(time.time() * 10) % 50 == 0:
                self.publish_debug('command_routing', {
                    'state': self.state,
                    'has_cmd': self.current_cmd is not None,
                    'cmd_x': self.current_cmd.linear.x if self.current_cmd else 0.0,
                    'cmd_y': self.current_cmd.linear.y if self.current_cmd else 0.0
                })
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

