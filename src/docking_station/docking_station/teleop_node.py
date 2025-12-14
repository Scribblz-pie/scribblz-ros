#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import math

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Declare parameter for base length (robot-specific)
        self.declare_parameter('base_length', 0.1)  # meters, default value
        self.base_length = self.get_parameter('base_length').get_parameter_value().double_value

        # Declare parameter for velocity scaling (linear and angular share the same scale)
        self.declare_parameter('max_velocity', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        # Separate scale for angular velocity to allow stronger turning
        self.declare_parameter('angular_scale', 10.0)
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
    
        
        self.motor_publisher = self.create_publisher(
            Twist,
            '/teleop/cmd_vel',
            1
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self.marker_publisher = self.create_publisher(
            Bool,
            '/marker',
            1
        )
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1
        )

        self.state_subscriber = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.current_state = 'docked'

        self.get_logger().info(f'teleop node initialized with base_length={self.base_length}m')
        
        # Parameter to select which Joy button drives the marker (e.g. keyboard "i")
        self.declare_parameter('marker_button_index', 4)
        self.marker_button_index = self.get_parameter('marker_button_index').get_parameter_value().integer_value
    
    def inverse_kinematics(self, x, y, psi):
        """
        Convert global velocities (x, y, psi) to motor velocities (v1, v2, v3)
        for a three-wheeled omni-directional robot.
        
        Args:
            x: velocity in x direction (m/s)
            y: velocity in y direction (m/s)
            psi: angular velocity (rad/s)
        
        Returns:
            tuple: (v1, v2, v3) motor velocities
        """
        l = self.base_length
        v1 = y + l * psi
        v2 = -0.5 * y - (math.sqrt(3) / 2.0) * x + l * psi
        v3 = -0.5 * y + (math.sqrt(3) / 2.0) * x + l * psi
        return v1, v2, v3

    def state_callback(self, msg):
        self.current_state = msg.data

    def joy_callback(self, msg):
        """
        Process joystick input from /joy topic.
        Mapping:
        - axes[0]: 1 = left, -1 = right (strafe)
        - axes[1]: 1 = up, -1 = down (strafe)
        - buttons[3]: 1 = rotate left
        - buttons[1]: 1 = rotate right
        - buttons[marker_button_index]: 1 = marker down (True), 0 = marker up (False)
        """
        # Extract joystick values
        strafe_left_right = msg.axes[0] if len(msg.axes) > 0 else 0.0  # 1=left, -1=right
        strafe_up_down = msg.axes[1] if len(msg.axes) > 1 else 0.0  # 1=up, -1=down
        rotate_left = msg.buttons[3] if len(msg.buttons) > 3 else 0  # rotate left
        rotate_right = msg.buttons[1] if len(msg.buttons) > 1 else 0  # rotate right

        # Convert to robot velocity commands
        # For omni robot: x = forward/backward, y = left/right
        # axes[0] = 1 means left, so positive y
        # axes[1] = 1 means up (forward), so positive x
        y_vel = strafe_left_right * self.max_velocity  # left/right (positive = left)
        x_vel = strafe_up_down * self.max_velocity     # forward/backward (positive = forward)

        # Angular velocity: rotate_left = -1, rotate_right = +1 (uses same scale as linear)
        psi_vel = (rotate_right - rotate_left) * self.max_velocity * self.angular_scale
        
        # Apply inverse kinematics
        v1, v2, v3 = self.inverse_kinematics(x_vel, y_vel, psi_vel)

        # Publish motor velocities as Twist message
        # Using linear.x, linear.y, linear.z for v1, v2, v3
        twist_msg = Twist()
        if self.current_state == 'teleop':
            twist_msg.linear.x = v1
            twist_msg.linear.y = v2
            twist_msg.linear.z = v3
            self.motor_publisher.publish(twist_msg)
            self.cmd_vel_publisher.publish(twist_msg)

        # Publish marker state (True when the configured button is pressed)
        marker_msg = Bool()
        marker_pressed = False
        if self.marker_button_index < len(msg.buttons):
            marker_pressed = msg.buttons[self.marker_button_index] == 1
        marker_msg.data = marker_pressed
        self.marker_publisher.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()