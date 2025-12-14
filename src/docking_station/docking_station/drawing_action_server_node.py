#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Empty, Bool, UInt32, Float64
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
import math
import time
import json


class DrawingActionServerNode(Node):
    def __init__(self):
        super().__init__('drawing_action_server')
        
        self.declare_parameter('waypoint_tolerance', 0.05)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('kp_linear', 2.0)  # Proportional gain for linear velocity
        self.declare_parameter('ki_linear', 0.0)  # Integral gain
        self.declare_parameter('kd_linear', 0.0)  # Derivative gain
        self.declare_parameter('target_fan_speed', 255)
        self.declare_parameter('fan_ramp_steps', 10)
        self.declare_parameter('fan_ramp_delay', 0.1)
        self.declare_parameter('enable_fan', True)  # Enable fan control
        self.declare_parameter('imu_reset_topic', '/imu/reset')
        self.declare_parameter('initialization_timeout', 5.0)
        
        self.tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        self.kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.target_fan_speed = self.get_parameter('target_fan_speed').get_parameter_value().integer_value
        self.fan_ramp_steps = self.get_parameter('fan_ramp_steps').get_parameter_value().integer_value
        self.fan_ramp_delay = self.get_parameter('fan_ramp_delay').get_parameter_value().double_value
        self.enable_fan = self.get_parameter('enable_fan').get_parameter_value().bool_value
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
        
        # Position feedback subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /robot_pose for position feedback')
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.get_logger().info('Subscribed to /imu/data for orientation feedback')
        
        # Command publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drawing/cmd_vel', 1)
        self.fan_pub = self.create_publisher(Int32, '/fan_speed', 1)
        self.imu_reset_pub = self.create_publisher(Empty, self.imu_reset_topic, 1)
        self.marker_pub = self.create_publisher(Bool, '/marker', 1)
        
        # Feedback publishers
        self.progress_pub = self.create_publisher(Float64, '/drawing/feedback/progress', 1)
        self.waypoint_index_pub = self.create_publisher(UInt32, '/drawing/feedback/waypoint_index', 1)
        
        # Result publishers
        self.success_pub = self.create_publisher(Bool, '/drawing/result/success', 1)
        self.completion_time_pub = self.create_publisher(Float64, '/drawing/result/completion_time', 1)
        
        # Debug publisher
        self.debug_pub = self.create_publisher(String, '/drawing/debug', 10)
        
        self.current_state = 'docked'
        self.active = False
        self.current_path = None
        self.stored_path = None  # Store latest path
        self.executing = False
        self.cancel_requested = False
        
        # Position feedback (from lidar/IMU)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_position_feedback = False
        
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.get_logger().info('drawing server initialized (topic-based)')
        
        # Test debug publisher immediately
        try:
            test_msg = String()
            test_msg.data = '{"event": "test", "timestamp": 0, "data": {"test": "debug_publisher_working"}}'
            self.debug_pub.publish(test_msg)
            self.get_logger().info('Debug publisher test message sent to /drawing/debug')
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug test: {e}')
        
        self.publish_debug('initialized', {
            'state': self.current_state,
            'active': self.active,
            'executing': self.executing,
            'debug_topic': '/drawing/debug'
        })
    
    def state_callback(self, msg):
        old_state = self.current_state
        new_state = msg.data
        self.current_state = new_state
        was_active = self.active
        self.active = (self.current_state == 'drawing')
        
        # Always publish state change debug
        self.publish_debug('state_change', {
            'old_state': old_state,
            'new_state': self.current_state,
            'was_active': was_active,
            'is_active': self.active,
            'has_stored_path': self.stored_path is not None,
            'executing': self.executing,
            'stored_path_waypoints': len(self.stored_path.poses) if self.stored_path else 0
        })
        
        # Log state change
        if old_state != new_state:
            self.get_logger().info(f'State changed: {old_state} -> {new_state}')
        
        if self.active and not was_active:
            self.get_logger().info('Entering drawing state')
            if self.stored_path is not None:
                self.get_logger().info('Starting execution of stored path')
                self.current_path = self.stored_path
                self.execute_path()
            else:
                self.get_logger().warn('Entered drawing state but no stored path available')
                self.publish_debug('no_path_available', {
                    'state': self.current_state,
                    'active': self.active,
                    'has_stored_path': False
                })
        elif not self.active and was_active:
            self.get_logger().info('Exiting drawing state')
            self.publish_debug('state_exit', {
                'reason': 'state changed from drawing',
                'new_state': self.current_state,
                'was_executing': self.executing
            })
            # Cancel execution if we exit drawing state
            if self.executing:
                self.cancel_requested = True
                self.executing = False
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
    
    def pose_callback(self, msg: PoseStamped):
        """Update current position from lidar."""
        old_x = self.current_x
        old_y = self.current_y
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        if not self.has_position_feedback:
            self.get_logger().info(f'First lidar position received: ({self.current_x:.3f}, {self.current_y:.3f})')
        
        self.has_position_feedback = True
        
        # Publish debug every position update
        self.publish_debug('position_update', {
            'x': self.current_x,
            'y': self.current_y,
            'old_x': old_x,
            'old_y': old_y,
            'from_lidar': True,
            'has_feedback': self.has_position_feedback
        })
    
    def imu_callback(self, msg: Imu):
        """Update current orientation from IMU."""
        # Extract yaw from quaternion
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.current_yaw = 2.0 * math.atan2(qz, qw)
    
    def cancel_callback(self, msg):
        self.cancel_requested = True
        # Raise marker when cancelled
        marker_up = Bool()
        marker_up.data = False
        self.marker_pub.publish(marker_up)
        self.get_logger().info('drawing cancellation requested')
    
    def path_callback(self, msg):
        self.stored_path = msg  # Always store the latest path
        
        # Log path details
        waypoint_details = []
        for i, pose in enumerate(msg.poses[:5]):  # First 5 waypoints
            waypoint_details.append({
                'index': i,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            })
        
        self.publish_debug('path_received', {
            'waypoint_count': len(msg.poses),
            'executing': self.executing,
            'active': self.active,
            'current_state': self.current_state,
            'first_waypoints': waypoint_details,
            'has_position_feedback': self.has_position_feedback,
            'current_position': {
                'x': self.current_x,
                'y': self.current_y
            } if self.has_position_feedback else None
        })
        
        self.get_logger().info(f'Path received: {len(msg.poses)} waypoints, state={self.current_state}, active={self.active}, executing={self.executing}')
        
        if self.executing:
            self.get_logger().info('new path received during execution - canceling current path and starting new one')
            self.publish_debug('path_cancel', {
                'reason': 'new_path_received',
                'was_executing': True
            })
            # Cancel current execution
            self.cancel_requested = True
            self.executing = False
            # Stop robot immediately
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            # Wait a moment for cancellation
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.active:
            self.get_logger().info('not in drawing state, storing path for later')
            self.publish_debug('path_stored', {
                'reason': 'not_active',
                'state': self.current_state
            })
            return
        
        # If active, start execution
        self.current_path = msg
        self.executing = True
        self.cancel_requested = False
        
        # Start execution
        self.get_logger().info(f'received path with {len(msg.poses)} waypoints')
        self.publish_debug('path_start', {
            'waypoint_count': len(msg.poses),
            'has_position_feedback': self.has_position_feedback
        })
        self.execute_path()
    
    def initialize_for_drawing(self):
        self.get_logger().info('initializing for drawing: resetting imu and ramping fan')
        
        # reset imu
        reset_msg = Empty()
        self.imu_reset_pub.publish(reset_msg)
        self.get_logger().info('imu reset command sent')
        
        if self.enable_fan:
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
        else:
            self.get_logger().info('fan control disabled')
        
        # Marker down for drawing
        marker_down = Bool()
        marker_down.data = True
        self.marker_pub.publish(marker_down)
        
        self.get_logger().info('initialization complete')
        return True
    
    def publish_debug(self, event, data):
        """Publish debug information."""
        try:
            debug_msg = String()
            debug_data = {
                'event': event,
                'timestamp': time.time(),
                'data': data
            }
            debug_msg.data = json.dumps(debug_data)
            self.debug_pub.publish(debug_msg)
            # Log important events at info level so they're visible
            if event in ['path_received', 'path_start', 'execution_start', 'control_loop_first_iteration', 'waypoint_reached', 'execution_complete']:
                self.get_logger().info(f'[DEBUG] {event}: {json.dumps(data, indent=2)}')
            else:
                self.get_logger().debug(f'[DEBUG] {event}: {data}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug message: {e}')
    
    def execute_path(self):
        if self.current_path is None or len(self.current_path.poses) == 0:
            self.get_logger().warn('no valid path to execute')
            self.publish_debug('execute_failed', {
                'reason': 'invalid_path',
                'path_is_none': self.current_path is None,
                'path_length': len(self.current_path.poses) if self.current_path else 0
            })
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
        
        # Marker down for drawing
        marker_down = Bool()
        marker_down.data = True
        self.marker_pub.publish(marker_down)
        
        start_time = time.time()
        current_waypoint_idx = 0
        dt = 1.0 / self.control_freq
        
        # Initialize position - use feedback if available, otherwise start at (0,0)
        if self.has_position_feedback:
            self.get_logger().info(f'Starting from lidar position: ({self.current_x:.3f}, {self.current_y:.3f})')
            self.publish_debug('execution_start', {
                'start_x': self.current_x,
                'start_y': self.current_y,
                'position_source': 'lidar',
                'waypoint_count': len(waypoints)
            })
        else:
            self.current_x = 0.0
            self.current_y = 0.0
            self.get_logger().warn('No lidar feedback - starting from (0,0) using dead reckoning')
            self.publish_debug('execution_start', {
                'start_x': 0.0,
                'start_y': 0.0,
                'position_source': 'dead_reckoning',
                'waypoint_count': len(waypoints),
                'warning': 'no_lidar_feedback'
            })
        
        # Reset PID
        self.prev_error = 0.0
        self.integral = 0.0
        
        waypoints = self.current_path.poses
        
        self.publish_debug('execution_loop_start', {
            'waypoint_count': len(waypoints),
            'has_position_feedback': self.has_position_feedback,
            'current_position': {'x': self.current_x, 'y': self.current_y},
            'active': self.active,
            'cancel_requested': self.cancel_requested
        })
        
        iteration_count = 0
        while current_waypoint_idx < len(waypoints) and self.active and not self.cancel_requested:
            iteration_count += 1
            
            # Debug first iteration
            if iteration_count == 1:
                self.get_logger().info(f'Control loop started - waypoint {current_waypoint_idx+1}/{len(waypoints)}')
                self.publish_debug('control_loop_first_iteration', {
                    'waypoint_index': current_waypoint_idx,
                    'target_x': waypoints[current_waypoint_idx].pose.position.x,
                    'target_y': waypoints[current_waypoint_idx].pose.position.y,
                    'current_x': self.current_x,
                    'current_y': self.current_y,
                    'has_feedback': self.has_position_feedback
                })
            
            # Check if we're stuck (too many iterations)
            if iteration_count > 10000:  # Safety limit
                self.get_logger().error('Execution loop exceeded iteration limit - possible infinite loop')
                self.publish_debug('execution_stuck', {
                    'iteration_count': iteration_count,
                    'waypoint_index': current_waypoint_idx,
                    'active': self.active,
                    'cancel_requested': self.cancel_requested
                })
                break
            
            target_pose = waypoints[current_waypoint_idx]
            target = target_pose.pose.position
            
            dx = target.x - self.current_x
            dy = target.y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.tolerance:
                self.publish_debug('waypoint_reached', {
                    'waypoint_index': current_waypoint_idx,
                    'total_waypoints': len(waypoints),
                    'current_position': {
                        'x': self.current_x,
                        'y': self.current_y
                    },
                    'desired_position': {
                        'x': target.x,
                        'y': target.y
                    },
                    'final_error': {
                        'dx': dx,
                        'dy': dy,
                        'distance': distance,
                        'tolerance': self.tolerance
                    },
                    'progress': {
                        'waypoints_completed': current_waypoint_idx + 1,
                        'percent_complete': ((current_waypoint_idx + 1) / len(waypoints)) * 100.0
                    }
                })
                current_waypoint_idx += 1
                # Reset PID for new waypoint
                self.prev_error = 0.0
                self.integral = 0.0
                if current_waypoint_idx >= len(waypoints):
                    self.publish_debug('path_complete', {
                        'total_waypoints': len(waypoints),
                        'completion_time': time.time() - start_time
                    })
                    break
                continue
            
            # PID control for linear velocity
            error = distance
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            linear_vel = self.kp_linear * error + self.ki_linear * self.integral + self.kd_linear * derivative
            linear_vel = max(0.0, min(self.max_linear, linear_vel))  # Clamp
            self.prev_error = error
            
            cmd = self.compute_velocity_to_waypoint(dx, dy, linear_vel)
            self.cmd_vel_pub.publish(cmd)
            
            # Log first few commands to verify they're being sent
            if iteration_count <= 3:
                self.get_logger().info(f'Iteration {iteration_count}: Publishing cmd_vel x={cmd.linear.x:.3f}, y={cmd.linear.y:.3f}, distance={distance:.3f}')
            
            # Publish comprehensive debug info every iteration
            self.publish_debug('control_update', {
                'waypoint_index': current_waypoint_idx,
                'total_waypoints': len(waypoints),
                'current_position': {
                    'x': self.current_x,
                    'y': self.current_y
                },
                'desired_position': {
                    'x': target.x,
                    'y': target.y
                },
                'error': {
                    'dx': dx,
                    'dy': dy,
                    'distance': distance,
                    'angle_rad': math.atan2(dy, dx),
                    'angle_deg': math.degrees(math.atan2(dy, dx))
                },
                'control': {
                    'linear_velocity': linear_vel,
                    'pid_error': error,
                    'pid_integral': self.integral,
                    'pid_derivative': derivative,
                    'kp': self.kp_linear,
                    'ki': self.ki_linear,
                    'kd': self.kd_linear
                },
                'command': {
                    'x': cmd.linear.x,
                    'y': cmd.linear.y,
                    'z': cmd.linear.z,
                    'angular_z': cmd.angular.z
                },
                'system': {
                    'has_lidar_feedback': self.has_position_feedback,
                    'active': self.active,
                    'executing': self.executing,
                    'cancel_requested': self.cancel_requested,
                    'dt': dt,
                    'elapsed_time': time.time() - start_time
                }
            })
            
            # Update position: use feedback if available, otherwise dead reckoning
            if self.has_position_feedback:
                # Position updated via pose_callback from lidar
                pass
            else:
                # Dead reckoning fallback
                self.current_x += cmd.linear.x * dt
                self.current_y += cmd.linear.y * dt
            
            # Publish feedback via topics
            progress_msg = Float64()
            progress_msg.data = float(current_waypoint_idx) / len(waypoints)
            self.progress_pub.publish(progress_msg)
            
            waypoint_idx_msg = UInt32()
            waypoint_idx_msg.data = current_waypoint_idx
            self.waypoint_index_pub.publish(waypoint_idx_msg)
            
            rclpy.spin_once(self, timeout_sec=dt)
            
            # Safety check - if we're not making progress and no feedback, warn
            if iteration_count % 100 == 0 and not self.has_position_feedback:
                self.publish_debug('no_position_feedback_warning', {
                    'iteration': iteration_count,
                    'elapsed_time': time.time() - start_time,
                    'waypoint_index': current_waypoint_idx,
                    'current_x': self.current_x,
                    'current_y': self.current_y
                })
        
        completion_time = time.time() - start_time
        
        self.publish_debug('execution_complete', {
            'reason': 'loop_exit',
            'waypoint_index': current_waypoint_idx,
            'total_waypoints': len(waypoints),
            'iterations': iteration_count,
            'completion_time': completion_time,
            'active': self.active,
            'cancel_requested': self.cancel_requested
        })
        
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
        
        # Raise marker when done or cancelled
        marker_up = Bool()
        marker_up.data = False
        self.marker_pub.publish(marker_up)
        
        time_msg = Float64()
        time_msg.data = completion_time
        self.completion_time_pub.publish(time_msg)
        
        # Raise marker when done or cancelled
        marker_up = Bool()
        marker_up.data = False
        self.marker_pub.publish(marker_up)
        
        self.executing = False
        self.current_path = None
    
    def compute_velocity_to_waypoint(self, dx, dy, linear_vel):
        cmd = Twist()
        
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.tolerance:
            return cmd
        
        angle = math.atan2(dy, dx)
        
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

