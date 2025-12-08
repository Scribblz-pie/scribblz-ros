#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
import socket

UDP_PORT = 54321  # must match Arduino port
BUFFER_SIZE = 64

class HeartbeatListener(Node):
    def __init__(self):
        super().__init__('heartbeat_listener')
        self.publisher_ = self.create_publisher(UInt32, 'arduino_heartbeat', 10)
        self.get_logger().info(f'Listening for UDP on port {UDP_PORT}')

        # Set up UDP socket (connectionless - listens for any sender)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', UDP_PORT))
        self.sock.setblocking(False)

        # Poll UDP at 20Hz
        self.timer = self.create_timer(0.05, self.poll_udp)

    def poll_udp(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(BUFFER_SIZE)
                msg_str = data.decode('utf-8').strip()
                if msg_str.startswith('HB '):
                    counter = int(msg_str.split(' ')[1])
                    msg = UInt32()
                    msg.data = counter
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Received HB {counter} from {addr[0]}')
        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

