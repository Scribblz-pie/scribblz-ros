#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
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
        
        # Publisher for motor velocities (v1, v2, v3)
        # Using Twist: linear.x=v1, linear.y=v2, linear.z=v3
        # Queue size 1 to prevent buffering old messages
        self.motor_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )
        
        # Subscriber for joystick input
        # Queue size 1 to only process latest message
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1
        )
        
        self.get_logger().info(f'Teleop node initialized with base_length={self.base_length}m')
    
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
    
    def joy_callback(self, msg):
        """
        Process joystick input from /joy topic.
        Mapping:
        - axes[0]: 1 = left, -1 = right (strafe)
        - axes[1]: 1 = up, -1 = down (strafe)
        - buttons[3]: 1 = rotate left
        - buttons[1]: 1 = rotate right
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
        psi_vel = (rotate_right - rotate_left) * self.max_velocity
        
        # Apply inverse kinematics
        v1, v2, v3 = self.inverse_kinematics(x_vel, y_vel, psi_vel)
        
        # Publish motor velocities as Twist message
        # Using linear.x, linear.y, linear.z for v1, v2, v3
        twist_msg = Twist()
        twist_msg.linear.x = v1
        twist_msg.linear.y = v2
        twist_msg.linear.z = v3
        
        self.motor_publisher.publish(twist_msg)
        
        # Debug logging (can be removed or set to debug level)
        self.get_logger().debug(
            f'Joy: LR={strafe_left_right:.2f}, UD={strafe_up_down:.2f}, rot_L={rotate_left}, rot_R={rotate_right} | '
            f'Vel: x={x_vel:.2f}, y={y_vel:.2f}, psi={psi_vel:.2f} | '
            f'Motors: v1={v1:.2f}, v2={v2:.2f}, v3={v3:.2f}'
        )

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

