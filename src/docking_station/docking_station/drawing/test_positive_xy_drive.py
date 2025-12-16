#!/usr/bin/env python3
"""
Test script to drive robot in positive X or positive Y direction.

Publishes velocity commands to /drawing/cmd_vel:
- linear.x = positive (forward, robot +x direction)
- linear.y = positive (left, robot +y direction)

ROS Parameter:
- direction: 'x' or 'y' (default: 'x')
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class PositiveXYDriveTest(Node):
    """Test node that drives robot in positive X or Y direction based on parameter."""
    
    def __init__(self):
        super().__init__('test_positive_xy_drive')
        
        # Declare parameter for direction selection
        self.declare_parameter('direction', 'x')  # 'x' or 'y'
        
        # Get direction parameter
        direction = self.get_parameter('direction').get_parameter_value().string_value.lower()
        if direction not in ['x', 'y']:
            self.get_logger().warn(f"Invalid direction '{direction}', defaulting to 'x'")
            direction = 'x'
        
        self.direction = direction
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 10)
        
        # Velocity value (m/s)
        self.velocity = 0.2
        
        # Duration to run (seconds)
        self.duration = 5.0
        
        # Control frequency (Hz)
        self.control_freq = 20.0
        self.dt = 1.0 / self.control_freq
        
        # Start timer
        self.start_time = None
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        direction_name = 'X (forward)' if self.direction == 'x' else 'Y (left)'
        self.get_logger().info(
            f'Starting positive {direction_name} drive test: '
            f'velocity={self.velocity} m/s, duration={self.duration} s'
        )
    
    def timer_callback(self):
        """Publish velocity commands periodically."""
        if self.start_time is None:
            self.start_time = time.time()
        
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.duration:
            # Stop robot
            self.publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info('Test complete - robot stopped')
            self.timer.cancel()
            return
        
        # Publish velocity based on direction parameter
        if self.direction == 'x':
            self.publish_velocity(self.velocity, 0.0, 0.0)
        else:  # direction == 'y'
            self.publish_velocity(0.0, self.velocity, 0.0)
        
        if int(elapsed * self.control_freq) % int(self.control_freq) == 0:  # Log once per second
            direction_name = 'X (forward)' if self.direction == 'x' else 'Y (left)'
            if self.direction == 'x':
                self.get_logger().info(
                    f'Driving in positive {direction_name}: vx={self.velocity:.2f} m/s, '
                    f'time={elapsed:.1f}/{self.duration:.1f} s'
                )
            else:
                self.get_logger().info(
                    f'Driving in positive {direction_name}: vy={self.velocity:.2f} m/s, '
                    f'time={elapsed:.1f}/{self.duration:.1f} s'
                )
    
    def publish_velocity(self, vx: float, vy: float, omega: float):
        """Publish velocity command.
        
        Args:
            vx: Forward velocity (m/s) - positive = forward (robot +x)
            vy: Lateral velocity (m/s) - positive = left (robot +y)
            omega: Angular velocity (rad/s)
        """
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PositiveXYDriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(0.0, 0.0, 0.0)  # Ensure robot stops
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

