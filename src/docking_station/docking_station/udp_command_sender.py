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

# --- CONFIGURATION ---
UDP_PORT = 54322
ARDUINO_IP = '255.255.255.255'  # Broadcast IP (Talks to everyone)

class UDPCommandSender(Node):
    def __init__(self):
        super().__init__('udp_command_sender')
        
        # Message counters for debugging
        self.motor_msg_count = 0
        self.fan_msg_count = 0
        self.marker_msg_count = 0
        self.marker_angle_msg_count = 0
        self.imu_msg_count = 0
        
        # Yaw integration state
        self.current_yaw = 0.0  # radians
        self.last_imu_time = None
        
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
        
        # Subscriber for marker state (Boolean)
        self.marker_subscriber = self.create_subscription(
            Bool,
            '/marker',
            self.marker_callback,
            1
        )
        
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
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # CRITICAL: Allow Broadcast packets (Fixes "Permission Denied" errors)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            # Bind to port 54322 to receive IMU data from Arduino
            # Arduino sends IMU to hardcoded IP:port (192.168.34.201:54322)
            self.sock.bind(('0.0.0.0', UDP_PORT))
            self.local_port = UDP_PORT
            
            self.sock.settimeout(0.1)  # Non-blocking read
            
            # Get local IP for logging
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(('8.8.8.8', 80))
                local_ip = s.getsockname()[0]
            except Exception:
                local_ip = 'unknown'
            finally:
                s.close()
            
            self.get_logger().info(f'UDP socket bound to port {self.local_port} on IP {local_ip}')
            self.get_logger().info(f'Arduino sends IMU to 192.168.34.201:54322 - ensure IP matches!')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            raise
        
        self.running = True
        
        # === LISTENER THREAD (Receives IMU data from Arduino) ===
        self.listener_thread = threading.Thread(target=self.udp_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        # === HEARTBEAT THREAD (Keeps connection alive) ===
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        self.get_logger().info(f'UDP command sender ready. Target: {ARDUINO_IP}:{UDP_PORT}')
    
    # ============================================================
    # COMMAND CALLBACKS (Send data TO Arduino)
    # ============================================================
    
    def cmd_callback(self, msg: Twist):
        """Send motor velocity commands to Arduino."""
        self.motor_msg_count += 1
        
        v1 = msg.linear.x
        v2 = msg.linear.y
        v3 = msg.linear.z
        
        cmd_str = f"CMD {v1:.4f} {v2:.4f} {v3:.4f}"
        self._send_udp(cmd_str, 'motor', self.motor_msg_count)
    
    def fan_callback(self, msg: Int32):
        """Send fan speed command to Arduino."""
        self.fan_msg_count += 1
        
        cmd_str = f"FAN {msg.data}"
        self._send_udp(cmd_str, 'fan', self.fan_msg_count)
    
    def marker_callback(self, msg: Bool):
        """Send marker state to Arduino."""
        self.marker_msg_count += 1
        
        marker_val = 1 if msg.data else 0
        cmd_str = f"MARKER {marker_val}"
        self._send_udp(cmd_str, 'marker', self.marker_msg_count)
    
    def marker_angle_callback(self, msg: Float64):
        """Send marker angle to Arduino."""
        self.marker_angle_msg_count += 1
        
        cmd_str = f"MARKER_ANGLE {msg.data:.2f}"
        self._send_udp(cmd_str, 'marker_angle', self.marker_angle_msg_count)
    
    def imu_reset_callback(self, msg: Empty):
        """Reset yaw integration to zero."""
        self.current_yaw = 0.0
        self.last_imu_time = None  # Reset timer to avoid large dt on next message
        self.get_logger().info('IMU yaw reset to 0°')
    
    def _send_udp(self, cmd_str, cmd_type, count):
        """Helper function to send UDP packets."""
        try:
            cmd_bytes = cmd_str.encode('utf-8')
            bytes_sent = self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
            self.get_logger().info(
                f'[{cmd_type} #{count}] Sent "{cmd_str}" ({bytes_sent} bytes)'
            )
        except Exception as e:
            self.get_logger().error(f'[{cmd_type} #{count}] UDP send error: {e}')
    
    # ============================================================
    # LISTENER THREAD (Receive data FROM Arduino)
    # ============================================================
    
    def heartbeat_loop(self):
        """Send periodic PING to register with Arduino."""
        while self.running and rclpy.ok():
            try:
                msg = b"PING"
                self.sock.sendto(msg, (ARDUINO_IP, UDP_PORT))
            except Exception as e:
                self.get_logger().warn(f"Heartbeat failed: {e}")
            
            time.sleep(1.0)
    
    def udp_listener(self):
        """Constantly listen for UDP packets from Arduino."""
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                if message.startswith('IMU'):
                    self.process_imu_message(message)
                else:
                    self.get_logger().debug(f"Received non-IMU message: {message}")
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"Listener error: {e}")
    
    def process_imu_message(self, message):
        """Parse 'IMU ax ay az gx gy gz', integrate yaw from gyro, and publish orientation."""
        try:
            parts = message.split()
            if len(parts) == 7:
                self.imu_msg_count += 1
                
                _, ax_str, ay_str, az_str, gx_str, gy_str, gz_str = parts
                
                # Convert gyro from deg/s to rad/s
                gz_rad_per_sec = float(gz_str) * math.pi / 180.0
                
                # Deadzone: filter out small gyro noise to prevent drift when stationary
                # 0.02 rad/s ≈ 1.15 deg/s - adjust if too sensitive/insensitive
                DEADZONE_THRESHOLD = 0.02  # rad/s
                if abs(gz_rad_per_sec) < DEADZONE_THRESHOLD:
                    gz_rad_per_sec = 0.0
                
                # Get current time
                current_time = self.get_clock().now()
                current_time_sec = current_time.nanoseconds / 1e9
                
                # Integrate yaw from gyro z-axis (yaw rate)
                if self.last_imu_time is not None:
                    dt = current_time_sec - self.last_imu_time
                    if 0 < dt < 1.0:  # dt between 0 and 1 second
                        self.current_yaw += gz_rad_per_sec * dt
                        # Normalize yaw to [0, 2π] (0-360 degrees)
                        self.current_yaw = self.current_yaw % (2.0 * math.pi)
                        if self.current_yaw < 0:
                            self.current_yaw += 2.0 * math.pi
                else:
                    self.current_yaw = 0.0
                
                self.last_imu_time = current_time_sec
                
                # Convert yaw to quaternion (roll=0, pitch=0, yaw=current_yaw)
                yaw_half = self.current_yaw / 2.0
                quaternion = [0.0, 0.0, math.sin(yaw_half), math.cos(yaw_half)]
                
                # Create and publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = current_time.to_msg()
                imu_msg.header.frame_id = 'imu_link'
                
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]
                
                imu_msg.angular_velocity.z = gz_rad_per_sec
                
                # Zero out unused fields
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = 0.0
                
                # Set covariances
                imu_msg.linear_acceleration_covariance = [0.0] * 9
                imu_msg.angular_velocity_covariance = [0.0] * 9
                imu_msg.orientation_covariance[0] = -1.0
                imu_msg.orientation_covariance[4] = -1.0
                imu_msg.orientation_covariance[8] = 0.01  # Yaw uncertainty
                
                self.imu_publisher.publish(imu_msg)
                
                if self.imu_msg_count % 50 == 0:
                    yaw_deg = math.degrees(self.current_yaw)
                    self.get_logger().info(f'[IMU #{self.imu_msg_count}] Yaw: {yaw_deg:.2f}°')
            else:
                self.get_logger().warn(f"IMU message has wrong number of parts: {len(parts)}, expected 7. Message: {message}")
                
        except (ValueError, NameError, AttributeError) as e:
            self.get_logger().error(f"Failed to process IMU data: {e}. Message: {message}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()

    def marker_callback(self, msg: Bool):
        """
        Receive marker state and send via UDP to Arduino.
        Format: "MARKER <0|1>" where 1 = down, 0 = up.
        """
        self.marker_msg_count += 1

        marker_val = 1 if msg.data else 0
        cmd_str = f"MARKER {marker_val}"
        cmd_bytes = cmd_str.encode('utf-8')

        try:
            bytes_sent = self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
            self.get_logger().info(
                f'[marker #{self.marker_msg_count}] Sent "{cmd_str}" ({bytes_sent} bytes) to {ARDUINO_IP}:{UDP_PORT}'
            )
        except Exception as e:
            self.get_logger().error(f'[marker #{self.marker_msg_count}] UDP send error: {e}')

    def marker_angle_callback(self, msg: Float64):
        """
        Receive marker angle (degrees) and send via UDP to Arduino.
        Format: "MARKER_ANGLE <deg>" where deg is typically 0-180.
        """
        self.marker_angle_msg_count += 1

        angle_val = msg.data
        cmd_str = f"MARKER_ANGLE {angle_val:.2f}"
        cmd_bytes = cmd_str.encode('utf-8')

        try:
            bytes_sent = self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
            self.get_logger().info(
                f'[marker_angle #{self.marker_angle_msg_count}] Sent "{cmd_str}" ({bytes_sent} bytes) to {ARDUINO_IP}:{UDP_PORT}'
            )
        except Exception as e:
            self.get_logger().error(f'[marker_angle #{self.marker_angle_msg_count}] UDP send error: {e}')

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