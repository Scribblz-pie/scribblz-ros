#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from docking_station.msg import BatteryStatus
from docking_station.srv import SetRobotState
from docking_station.action import ExecuteDrawing, DockRobot


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = 'docked'
        self.valid_states = ['docked', 'drawing', 'returning_to_docking_station', 'teleop', 'error', 'recovering']
        
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        self.set_state_srv = self.create_service(
            SetRobotState,
            '/set_robot_state',
            self.set_state_callback
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
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
        
        self.drawing_action_client = ActionClient(self, ExecuteDrawing, '/execute_drawing')
        self.docking_action_client = ActionClient(self, DockRobot, '/dock_robot')
        
        self.current_cmd = None
        self.low_battery = False
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('state machine initialized, starting in docked state')
    
    def set_state_callback(self, request, response):
        new_state = request.state
        if new_state in self.valid_states:
            if self.is_valid_transition(self.state, new_state):
                self.state = new_state
                self.get_logger().info(f'state changed to: {new_state}')
                response.success = True
                response.message = f'state changed to {new_state}'
                
                if new_state == 'drawing':
                    self.start_drawing_action()
                elif new_state == 'returning_to_docking_station':
                    self.start_docking_action()
            else:
                response.success = False
                response.message = f'invalid transition from {self.state} to {new_state}'
        else:
            response.success = False
            response.message = f'invalid state: {new_state}'
        return response
    
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
        self.low_battery = msg.low_battery
        if self.low_battery and self.state == 'drawing':
            self.get_logger().warn('low battery detected, transitioning to returning_to_docking_station')
            self.state = 'returning_to_docking_station'
            self.start_docking_action()
    
    def drawing_cmd_callback(self, msg):
        if self.state == 'drawing':
            self.current_cmd = msg
    
    def docking_cmd_callback(self, msg):
        if self.state == 'returning_to_docking_station':
            self.current_cmd = msg
    
    def teleop_cmd_callback(self, msg):
        if self.state == 'teleop':
            self.current_cmd = msg
    
    def start_drawing_action(self):
        pass
    
    def start_docking_action(self):
        pass
    
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

