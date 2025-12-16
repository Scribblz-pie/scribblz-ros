#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Int32
import math
import time
import threading

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available. GPIO control will be simulated.")


class TetherControllerNode(Node):
    def __init__(self):
        super().__init__('tether_controller')
        
        # Parameters - Wire exit height
        self.declare_parameter('wire_exit_height', 0.5)  # Height where wire leaves (meters)
        
        # Parameters - Physical properties
        self.declare_parameter('spool_diameter', 0.1)  # Spool diameter in meters
        self.declare_parameter('gear_ratio', 1.0)  # Motor rotations per spool rotation
        self.declare_parameter('steps_per_revolution', 200)  # Stepper steps per revolution
        
        # Parameters - GPIO pins
        self.declare_parameter('gpio_step_pin', 18)
        self.declare_parameter('gpio_dir_pin', 19)
        
        # Parameters - Control
        self.declare_parameter('control_frequency', 20.0)  # Hz
        self.declare_parameter('max_step_rate', 1000)  # steps/second
        self.declare_parameter('step_deadband', 1)  # Don't move if error < this many steps
        
        # Parameters - Calibration
        self.declare_parameter('initial_wire_length', 0.0)  # Wire length at calibration (meters)
        self.declare_parameter('calibration_robot_x', 0.0)
        self.declare_parameter('calibration_robot_y', 0.0)
        
        # Parameters - Safety
        self.declare_parameter('pose_timeout', 1.0)  # Seconds without pose before stopping
        
        # Get parameters
        self.wire_height = self.get_parameter('wire_exit_height').get_parameter_value().double_value
        
        self.spool_diameter = self.get_parameter('spool_diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.steps_per_rev = self.get_parameter('steps_per_revolution').get_parameter_value().integer_value
        
        self.step_pin = self.get_parameter('gpio_step_pin').get_parameter_value().integer_value
        self.dir_pin = self.get_parameter('gpio_dir_pin').get_parameter_value().integer_value
        
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.max_step_rate = self.get_parameter('max_step_rate').get_parameter_value().double_value
        self.step_deadband = self.get_parameter('step_deadband').get_parameter_value().integer_value
        
        self.initial_wire_length = self.get_parameter('initial_wire_length').get_parameter_value().double_value
        self.calib_x = self.get_parameter('calibration_robot_x').get_parameter_value().double_value
        self.calib_y = self.get_parameter('calibration_robot_y').get_parameter_value().double_value
        
        self.pose_timeout = self.get_parameter('pose_timeout').get_parameter_value().double_value
        
        # Calculate calibration distance (3D) - cache this value
        # Since spool is at origin (0,0), use calibration position directly
        calib_horizontal = math.sqrt(self.calib_x**2 + self.calib_y**2)
        self.calib_wire_length = math.sqrt(calib_horizontal**2 + self.wire_height**2)
        
        # Calculate initial step offset
        # At calibration position, if user provided initial_wire_length (measured),
        # we use that to calculate the step offset. Otherwise, we assume the calculated
        # wire length is correct and start at step 0.
        if self.initial_wire_length > 0:
            # User provided measured wire length at calibration position
            # The difference tells us the step offset
            wire_length_offset = self.initial_wire_length - self.calib_wire_length
            self.initial_steps = self._wire_length_to_steps(wire_length_offset)
        else:
            # Use calculated wire length at calibration position (assume it's correct)
            self.initial_steps = 0
        
        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_step_position = self.initial_steps
        self.last_pose_time = None
        self.has_pose = False
        
        # Publishers
        self.length_pub = self.create_publisher(Float64, '/tether/length', 10)
        self.target_steps_pub = self.create_publisher(Int32, '/tether/target_steps', 10)
        self.current_steps_pub = self.create_publisher(Int32, '/tether/current_steps', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # GPIO setup
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.setup(self.dir_pin, GPIO.OUT)
            GPIO.output(self.step_pin, GPIO.LOW)
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.get_logger().debug(f'GPIO initialized: step_pin={self.step_pin}, dir_pin={self.dir_pin}')
        else:
            self.get_logger().warn('GPIO not available - running in simulation mode')
        
        # Motor control thread
        self.motor_running = False
        self.motor_lock = threading.Lock()
        self.target_step_position = self.initial_steps
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.control_freq, self.control_loop)
        
        self.get_logger().info(
            f'Tether controller initialized:\n'
            f'  Spool at origin (0, 0) - wire exit height: {self.wire_height:.3f}m\n'
            f'  Spool diameter: {self.spool_diameter:.3f}m\n'
            f'  Gear ratio: {self.gear_ratio:.3f}\n'
            f'  Steps per revolution: {self.steps_per_rev}\n'
            f'  Initial steps: {self.initial_steps}\n'
            f'  Control frequency: {self.control_freq}Hz'
        )
    
    def _wire_length_to_steps(self, wire_length: float) -> int:
        """Convert wire length change to step position.
        
        Args:
            wire_length: Change in wire length from calibration (meters)
            
        Returns:
            Step position relative to calibration
        """
        if abs(wire_length) < 1e-6:
            return 0
        
        # Calculate spool rotations needed
        spool_circumference = math.pi * self.spool_diameter
        spool_rotations = wire_length / spool_circumference
        
        # Calculate motor rotations (accounting for gear ratio)
        motor_rotations = spool_rotations * self.gear_ratio
        
        # Convert to steps
        steps = int(motor_rotations * self.steps_per_rev)
        
        return steps
    
    def _calculate_3d_distance(self, robot_x: float, robot_y: float) -> float:
        """Calculate 3D wire length from origin (docking station) to robot.
        
        Since the spool is at the origin (0,0) on top of the lidar,
        we use robot position directly.
        
        Args:
            robot_x: Robot X position
            robot_y: Robot Y position
            
        Returns:
            Wire length in meters
        """
        # Horizontal distance from origin
        horizontal_dist = math.sqrt(robot_x**2 + robot_y**2)
        
        # 3D wire length (hypotenuse of right triangle)
        wire_length = math.sqrt(horizontal_dist**2 + self.wire_height**2)
        
        return wire_length
    
    def pose_callback(self, msg: PoseStamped):
        """Update robot position from pose message."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.last_pose_time = self.get_clock().now()
        self.has_pose = True
    
    def control_loop(self):
        """Main control loop - calculates target steps and moves motor."""
        # Check for pose timeout
        if self.has_pose and self.last_pose_time is not None:
            time_since_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
            if time_since_pose > self.pose_timeout:
                self.get_logger().warn(f'Pose timeout ({time_since_pose:.2f}s) - stopping motor')
                self.has_pose = False
                self.target_step_position = self.current_step_position  # Stop movement
        
        if not self.has_pose:
            return
        
        # Calculate current wire length (3D)
        wire_length = self._calculate_3d_distance(self.current_x, self.current_y)
        
        # Calculate wire length change from calibration position
        wire_length_change = wire_length - self.calib_wire_length
        
        # Convert to target step position
        steps_from_calib = self._wire_length_to_steps(wire_length_change)
        target_steps = self.initial_steps + steps_from_calib
        
        # Update target
        with self.motor_lock:
            self.target_step_position = target_steps
        
        # Publish diagnostics
        length_msg = Float64()
        length_msg.data = wire_length
        self.length_pub.publish(length_msg)
        
        target_msg = Int32()
        target_msg.data = target_steps
        self.target_steps_pub.publish(target_msg)
        
        current_msg = Int32()
        current_msg.data = self.current_step_position
        self.current_steps_pub.publish(current_msg)
        
        # Move motor toward target
        self._move_motor_toward_target()
    
    def _move_motor_toward_target(self):
        """Move motor toward target step position."""
        with self.motor_lock:
            target = self.target_step_position
            current = self.current_step_position
        
        steps_to_move = target - current
        
        # Deadband - don't move if error is too small
        if abs(steps_to_move) <= self.step_deadband:
            return
        
        # Determine direction
        direction = steps_to_move > 0  # True = extend wire, False = retract
        
        # Limit step rate - move in small increments per control cycle
        max_steps_per_cycle = int(self.max_step_rate / self.control_freq)
        steps_this_cycle = min(abs(steps_to_move), max_steps_per_cycle)
        
        # Move motor
        if GPIO_AVAILABLE:
            GPIO.output(self.dir_pin, GPIO.HIGH if direction else GPIO.LOW)
            
            # Calculate step delay (microseconds)
            if steps_this_cycle > 0:
                step_delay_us = int(1e6 / self.max_step_rate)
                
                for _ in range(steps_this_cycle):
                    GPIO.output(self.step_pin, GPIO.HIGH)
                    time.sleep(step_delay_us / 1e6)
                    GPIO.output(self.step_pin, GPIO.LOW)
                    time.sleep(step_delay_us / 1e6)
                    
                    # Update position
                    if direction:
                        self.current_step_position += 1
                    else:
                        self.current_step_position -= 1
        else:
            # Simulation mode - just update position
            if direction:
                self.current_step_position += steps_this_cycle
            else:
                self.current_step_position -= steps_this_cycle
            
            self.get_logger().debug(
                f'Simulated: moved {steps_this_cycle} steps '
                f'({"extend" if direction else "retract"}), '
                f'current={self.current_step_position}, target={target}'
            )
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TetherControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

