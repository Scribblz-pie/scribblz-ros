#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import socket

UDP_PORT = 54322  # Different port from heartbeat (54321)
ARDUINO_IP = '255.255.255.255'  # Broadcast; set to Arduino IP for unicast if desired

class UDPCommandSender(Node):
    def __init__(self):
        super().__init__('udp_command_sender')
        
        # Message counters for debugging
        self.motor_msg_count = 0
        self.fan_msg_count = 0
        
        # Subscriber for motor velocity commands
        # Queue size 1 to only process latest message (prevents buffering)
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            1
        )

        # Subscriber for fan speed (integer) commands
        # Queue size 1 to avoid buffering
        self.fan_subscriber = self.create_subscription(
            Int32,
            '/fan_speed',
            self.fan_callback,
            1
        )
        
        # Set up UDP socket for sending commands
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
            self.get_logger().info(f'UDP socket created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            raise
        
        self.get_logger().info(f'UDP command sender initialized, sending to {ARDUINO_IP}:{UDP_PORT}')
    
    def cmd_callback(self, msg: Twist):
        """
        Receive motor velocity commands and send via UDP to Arduino.
        Format: "CMD v1 v2 v3" where v1, v2, v3 are motor velocities.
        """
        self.motor_msg_count += 1
        
        # Extract motor velocities
        v1 = msg.linear.x
        v2 = msg.linear.y
        v3 = msg.linear.z
        
        # Format message: "CMD v1 v2 v3"
        cmd_str = f"CMD {v1:.4f} {v2:.4f} {v3:.4f}"
        cmd_bytes = cmd_str.encode('utf-8')
        
        try:
            # Send via UDP broadcast
            bytes_sent = self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
            self.get_logger().info(
                f'[motor #{self.motor_msg_count}] Sent "{cmd_str}" ({bytes_sent} bytes) to {ARDUINO_IP}:{UDP_PORT}'
            )
        except Exception as e:
            self.get_logger().error(f'[motor #{self.motor_msg_count}] UDP send error: {e}')

    def fan_callback(self, msg: Int32):
        """
        Receive fan speed command (integer) and send via UDP to Arduino.
        Format: "FAN <value>"
        """
        self.fan_msg_count += 1

        fan_val = msg.data
        cmd_str = f"FAN {fan_val}"
        cmd_bytes = cmd_str.encode('utf-8')

        try:
            bytes_sent = self.sock.sendto(cmd_bytes, (ARDUINO_IP, UDP_PORT))
            self.get_logger().info(
                f'[fan #{self.fan_msg_count}] Sent "{cmd_str}" ({bytes_sent} bytes) to {ARDUINO_IP}:{UDP_PORT}'
            )
        except Exception as e:
            self.get_logger().error(f'[fan #{self.fan_msg_count}] UDP send error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UDPCommandSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

