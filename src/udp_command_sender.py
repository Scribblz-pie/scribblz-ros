#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Float64, Empty
from sensor_msgs.msg import Imu
import socket

        # Message counters for debugging
        self.motor_msg_count = 0
        self.fan_msg_count = 0
        self.marker_msg_count = 0
        self.marker_angle_msg_count = 0
        self.imu_msg_count = 0
        
        # Subscriber for explicit marker angle commands (degrees 0-180)
        self.marker_angle_subscriber = self.create_subscription(
            Float64,
            '/marker_angle',
            self.marker_angle_callback,
            1
        )
        
        # === PUBLISHERS (Data FROM Arduino) ===
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # Set up UDP socket for sending commands
