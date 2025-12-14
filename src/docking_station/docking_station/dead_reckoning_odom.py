#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import time

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning_odom')
        
        # 1. Initialize State at the Dock
        # (Must match your waypoints.json dock position)
        self.x = 1.05
        self.y = -0.15
        self.theta = 0.0
        
        self.last_time = self.get_clock().now()
        self.current_cmd = Twist()
        
        # 2. Subscribe to REAL IMU (for accurate rotation)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # 3. Subscribe to OWN Commands (to guess position)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 4. Publish "Fake" Pose (so the controller is happy)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        
        # Run loop at 50Hz
        self.create_timer(0.02, self.update_odometry)
        self.get_logger().info("Dead Reckoning Started. Using IMU for theta, Commands for X/Y.")

    def imu_callback(self, msg):
        # Trust the Gyroscope for orientation!
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.theta = yaw

    def cmd_callback(self, msg):
        # Save the command we just sent to the motors
        self.current_cmd = msg

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- THE MATH ---
        # 1. Get velocities commanded in ROBOT frame
        vx_robot = self.current_cmd.linear.x
        vy_robot = self.current_cmd.linear.y
        
        # 2. Rotate to WORLD frame using current Theta
        # (Standard Rotation Matrix)
        vx_world = vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)
        vy_world = vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)
        
        # 3. Integrate Position (Dead Reckoning)
        self.x += vx_world * dt
        self.y += vy_world * dt
        
        # --- PUBLISH ---
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        
        q = quaternion_from_euler(0, 0, self.theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()