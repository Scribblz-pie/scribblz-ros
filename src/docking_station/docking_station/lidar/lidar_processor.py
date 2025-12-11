#!/usr/bin/env python3

"""
Main lidar processor node that orchestrates packet parsing and coordinate calculation.

This module:
- Manages ROS2 node lifecycle and parameters
- Coordinates between packet parsing and circle fitting
- Publishes lidar scans and robot pose
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import threading
import math
from collections import deque

from .packet_parser import LidarReceiver, ScanAssembler
from .circle_fitter import CircleFitter


class LidarProcessor(Node):
    """
    Main processor that orchestrates lidar data processing pipeline.
    
    Receives UDP packets, assembles scans, fits circles, and publishes results.
    """
    
    def __init__(self):
        super().__init__('lidar_pose')

        # Network parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 6794)
        self.declare_parameter('print_interval', 2.0)

        # Scan assembly parameters
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('start_filter_angle', 0.0)
        self.declare_parameter('end_filter_angle', 360.0)

        # Frame IDs
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scan_frame_id', 'lidar')

        # Circle fit configuration
        self.declare_parameter('expected_radius_cm', 3.0)
        self.declare_parameter('radius_tolerance_cm', 5.0)
        self.declare_parameter('min_circle_points', 3)
        self.declare_parameter('ransac_iterations', 50)
        self.declare_parameter('inlier_threshold_cm', 2.0)
        self.declare_parameter('min_detection_distance_cm', 5.0)
        self.declare_parameter('max_detection_distance_cm', 500.0)

        # Load parameters
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.angle_tol = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.start_filter = self.get_parameter('start_filter_angle').get_parameter_value().double_value
        self.end_filter = self.get_parameter('end_filter_angle').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_frame_id = self.get_parameter('scan_frame_id').get_parameter_value().string_value
        self.print_interval = self.get_parameter('print_interval').get_parameter_value().double_value

        # Circle fit configuration dict
        self.cfg = {
            "expected_radius_cm": self.get_parameter('expected_radius_cm').get_parameter_value().double_value,
            "radius_tolerance_cm": self.get_parameter('radius_tolerance_cm').get_parameter_value().double_value,
            "min_circle_points": self.get_parameter('min_circle_points').get_parameter_value().integer_value,
            "ransac_iterations": self.get_parameter('ransac_iterations').get_parameter_value().integer_value,
            "inlier_threshold_cm": self.get_parameter('inlier_threshold_cm').get_parameter_value().double_value,
            "min_detection_distance_cm": self.get_parameter('min_detection_distance_cm').get_parameter_value().double_value,
            "max_detection_distance_cm": self.get_parameter('max_detection_distance_cm').get_parameter_value().double_value,
        }

        # ROS2 publishers
        qos = QoSProfile(depth=10)
        self.scan_pub = self.create_publisher(LaserScan, '/lidar/scan', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', qos)

        # Initialize components
        self.buffer = deque(maxlen=64)
        self.stop_event = threading.Event()
        self.receiver = LidarReceiver(self.host, self.port, self.buffer, self.stop_event, self.print_interval)
        self.assembler = ScanAssembler(self.angle_tol)
        self.circle_fitter = CircleFitter()

        # Start receiver thread and processing timer
        self.receiver.start()
        self.timer = self.create_timer(0.02, self.process_buffer)

        self.get_logger().info(f'lidar pose node listening on {self.host}:{self.port}')

    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.stop_event.set()
        if self.receiver.is_alive():
            self.receiver.join(timeout=2.0)
        super().destroy_node()

    def angle_in_filter(self, a):
        """
        Check if angle is within the filter range.
        
        Args:
            a: Angle in degrees
            
        Returns:
            True if angle should be included
        """
        if self.start_filter <= self.end_filter:
            return self.start_filter <= a <= self.end_filter
        return a >= self.start_filter or a <= self.end_filter

    def process_buffer(self):
        """
        Process buffered packets and publish scans/poses.
        
        Called periodically by ROS2 timer.
        """
        # Assemble packets into complete scans
        updated = False
        scan = None
        while self.buffer:
            pkt = self.buffer.popleft()
            res = self.assembler.add_packet(*pkt)
            if res:
                scan = res
                updated = True

        if not updated or scan is None:
            return

        ts, addr, angles, distances, intensities = scan

        # Filter angles
        filt_angles = []
        filt_distances = []
        for a, d in zip(angles, distances):
            if self.angle_in_filter(a):
                filt_angles.append(a)
                filt_distances.append(d)

        if len(filt_angles) == 0:
            return

        # Publish scan
        self.publish_scan(filt_angles, filt_distances, ts)

        # Fit circle and publish pose
        circle_fit = self.circle_fitter.fit_circle_ransac(filt_angles, filt_distances, self.cfg)
        if circle_fit:
            self.publish_pose(circle_fit, ts)

    def publish_scan(self, angles_deg, distances_m, ts):
        """
        Publish a LaserScan message.
        
        Args:
            angles_deg: List of angles in degrees
            distances_m: List of distances in meters
            ts: Timestamp (unused, kept for API compatibility)
        """
        sorted_pairs = sorted(zip(angles_deg, distances_m), key=lambda x: x[0])
        angles_deg_sorted = [a for a, _ in sorted_pairs]
        ranges = [d for _, d in sorted_pairs]

        if not angles_deg_sorted:
            return

        angle_min = math.radians(angles_deg_sorted[0])
        angle_max = math.radians(angles_deg_sorted[-1])
        if len(angles_deg_sorted) > 1:
            angle_increment = (angle_max - angle_min) / (len(angles_deg_sorted) - 1)
        else:
            angle_increment = 0.0

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.scan_frame_id
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = max(ranges) if ranges else 0.0
        msg.ranges = ranges
        msg.intensities = []
        self.scan_pub.publish(msg)

    def publish_pose(self, circle_fit, ts):
        """
        Publish robot pose based on detected circle.
        
        Args:
            circle_fit: Tuple of (center_x_cm, center_y_cm, radius_cm, inlier_count)
            ts: Timestamp (unused, kept for API compatibility)
        """
        cx_cm, cy_cm, r, inliers = circle_fit
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = cx_cm / 100.0  # Convert cm to meters
        pose.pose.position.y = cy_cm / 100.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
