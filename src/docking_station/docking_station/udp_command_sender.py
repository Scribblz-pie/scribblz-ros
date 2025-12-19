#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Float64, Empty
from sensor_msgs.msg import Imu
import socket
import threading
import math
import time
import os
import csv

# --- CONFIGURATION ---
UDP_PORT = 54322
# Broadcast allows sending to the Arduino without knowing its dynamic IP.
# However, the Arduino will ONLY reply to the specific IP defined in its firmware (192.168.34.201).
ARDUINO_IP = '255.255.255.255'


class UDPCommandSender(Node):
    def __init__(self):
        super().__init__('udp_command_sender')
        
        # Message counters for debugging
        self.motor_msg_count = 0
        self.fan_msg_count = 0
        self.calib_msg_count = 0
        self.imu_msg_count = 0
        
        # IMU integration state
        self.current_yaw = 0.0
        self.last_imu_time = None

        # IMU logging (orientation.z over time)
        self.imu_log_path = os.path.expanduser('~/imu_orientation_log.csv')
        self.imu_log_header_written = False
        
        # === SUBSCRIBERS (Commands to send to Arduino) ===
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            1
        )
        
        self.fan_subscriber = self.create_subscription(
            Int32,
            '/fan_speed',
            self.fan_callback,
            1
        )
        
        # Reset Gyro/Yaw Command
        self.reset_subscriber = self.create_subscription(
            Empty,
            '/imu/reset',
            self.imu_reset_callback,
            1
        )
        
        # Marker control: /marker (Bool) -> MARKER <pulse_us> for servo
        self.marker_subscriber = self.create_subscription(
            Bool, '/marker', self.marker_callback, 1
        )
        
        # === IMU FILTER STATE ===
        # Simple first-order low-pass (exponential moving average) for acceleration
        self.acc_x_filtered = 0.0
        self.acc_y_filtered = 0.0
        self.acc_z_filtered = 0.0
        # Smoothing factors in (0, 1]; lower = more smoothing (more lag, less noise)
        self.acc_filter_alpha = 0.2

        # === PUBLISHERS (Data FROM Arduino) ===
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # Set up UDP socket
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            # Bind to local port to receive data from Arduino
            # Note: Ensure this machine has the IP 192.168.34.201, 
            # otherwise the Arduino will not send packets back.
            self.sock.bind(('192.168.50.1', UDP_PORT))  # Use your actual wlan1 IP
            self.sock.settimeout(0.1)  # Non-blocking read
            
            self.get_logger().debug(f'UDP socket bound to port {UDP_PORT}')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            raise
        
        self.running = True
        
        # === THREADS ===
        self.listener_thread = threading.Thread(target=self.udp_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        self.get_logger().debug('UDP Bridge Ready.')
    
    # ============================================================
    # COMMAND CALLBACKS (Send data TO Arduino)
    # ============================================================
    
    def cmd_callback(self, msg: Twist):
        """Send motor velocity commands: CMD v1 v2 v3"""
        self.motor_msg_count += 1
        v1 = msg.linear.x
        v2 = msg.linear.y
        v3 = msg.linear.z
        cmd_str = f"CMD {v1:.4f} {v2:.4f} {v3:.4f}"
        self._send_udp(cmd_str)
    
    def fan_callback(self, msg: Int32):
        """Send fan speed command: FAN <microseconds>"""
        self.fan_msg_count += 1
        cmd_str = f"FAN {msg.data}"
        self._send_udp(cmd_str)

    def imu_reset_callback(self, msg: Empty):
        """Reset integrated yaw to zero"""
        self.calib_msg_count += 1
        self.current_yaw = 0.0
        self.last_imu_time = None
        self.get_logger().debug('IMU yaw reset to 0°')

    def marker_callback(self, msg: Bool):
        """Send marker servo pulse in microseconds via MARKER command.

        The Arduino firmware expects:
            MARKER <pulse_us>
        where <pulse_us> is a servo pulse width in microseconds, typically 1000–2000.

        For compatibility with the /marker Bool topic:
        - True  -> marker down (max pulse)
        - False -> marker up (min pulse)
        """
        pulse_min_us = 1000
        pulse_max_us = 2000
        pulse = pulse_max_us if msg.data else pulse_min_us
        self._send_udp(f"MARKER {pulse}")
    
    def _send_udp(self, cmd_str):
        """Helper to send UDP packets."""
        try:
            cmd_bytes = cmd_str.encode('utf-8')
            self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
        except Exception as e:
            self.get_logger().error(f'UDP send error: {e}')
    
    # ============================================================
    # LISTENER & PARSING (Receive data FROM Arduino)
    # ============================================================
    
    def heartbeat_loop(self):
        """Periodic PING to keep connection alive (if needed by future firmware)."""
        while self.running and rclpy.ok():
            self._send_udp("PING")
            time.sleep(1.0)
    
    def udp_listener(self):
        """Listen for UDP packets."""
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                # Arduino Format: "IMU ax ay az gx gy gz"
                if message.startswith('IMU'):
                    self.process_imu_message(message)
                else:
                    self.get_logger().debug(f"Unknown message: {message}")
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"Listener error: {e}")
    
    def process_imu_message(self, message):
        """
        Parses 'IMU ax ay az gx gy gz' format from Arduino
        Integrates gyro Z to compute yaw orientation
        """
        try:
            parts = message.split()
            if len(parts) != 7:  # "IMU" + 6 values
                return
            
            self.imu_msg_count += 1
            
            # Parse raw values
            ax_raw = float(parts[1])
            ay_raw = float(parts[2])
            az_raw = float(parts[3])
            gx = float(parts[4])
            gy = float(parts[5])
            gz = float(parts[6])  # deg/s
            
            # Convert gyro Z from deg/s to rad/s
            gz_rad_per_sec = gz * math.pi / 180.0

            # Deadzone to prevent drift when stationary
            DEADZONE_THRESHOLD = 0.02  # rad/s (~1.15 deg/s)
            if abs(gz_rad_per_sec) < DEADZONE_THRESHOLD:
                gz_rad_per_sec = 0.0
            
            # Get current time
            current_time = self.get_clock().now()
            current_time_sec = current_time.nanoseconds / 1e9
            
            # Integrate yaw from gyro Z-axis
            if self.last_imu_time is not None:
                dt = current_time_sec - self.last_imu_time
                if 0 < dt < 1.0:  # Sanity check: dt between 0 and 1 second
                    self.current_yaw += gz_rad_per_sec * dt
                    # Normalize to [-π, π] to match path_follower expectations
                    while self.current_yaw > math.pi:
                        self.current_yaw -= 2.0 * math.pi
                    while self.current_yaw < -math.pi:
                        self.current_yaw += 2.0 * math.pi
            else:
                self.current_yaw = 0.0
            
            self.last_imu_time = current_time_sec
            
            # Convert yaw to quaternion
            half_yaw = self.current_yaw / 2.0
            qz = math.sin(half_yaw)
            qw = math.cos(half_yaw)
            
            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Orientation (quaternion)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            
            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = gz_rad_per_sec
            
            # Low-pass filter linear acceleration (optional, reduces vibration spikes)
            self.acc_x_filtered += self.acc_filter_alpha * (ax_raw - self.acc_x_filtered)
            self.acc_y_filtered += self.acc_filter_alpha * (ay_raw - self.acc_y_filtered)
            self.acc_z_filtered += self.acc_filter_alpha * (az_raw - self.acc_z_filtered)

            # Linear acceleration (convert g to m/s²) using filtered values
            imu_msg.linear_acceleration.x = self.acc_x_filtered * 9.81
            imu_msg.linear_acceleration.y = self.acc_y_filtered * 9.81
            imu_msg.linear_acceleration.z = self.acc_z_filtered * 9.81
            
            # Covariances
            imu_msg.orientation_covariance = [0.0] * 9
            imu_msg.orientation_covariance[8] = 0.01  # Yaw uncertainty
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance = [0.0] * 9

            # Log yaw (self.current_yaw) to CSV for offline analysis
            try:
                file_exists = os.path.exists(self.imu_log_path)
                with open(self.imu_log_path, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    if not file_exists or os.path.getsize(self.imu_log_path) == 0:
                        # Header: time [s], yaw [rad], yaw_deg [deg]
                        writer.writerow(['time_sec', 'yaw_rad', 'yaw_deg'])
                    yaw_rad = self.current_yaw
                    yaw_deg = math.degrees(yaw_rad)
                    writer.writerow([current_time_sec, yaw_rad, yaw_deg])
            except Exception as e:
                # Non-fatal: just log once in debug
                self.get_logger().debug(f'Failed to write IMU log: {e}')

            self.imu_publisher.publish(imu_msg)
            
            if self.imu_msg_count % 50 == 0:
                yaw_deg = math.degrees(self.current_yaw)
                self.get_logger().debug(f'[IMU #{self.imu_msg_count}] Yaw: {yaw_deg:.2f}°')
        
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"IMU Parse Error: {e}")

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPCommandSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()