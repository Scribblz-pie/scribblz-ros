#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        self.declare_parameter('battery_topic', '/battery_voltage')
        self.declare_parameter('low_battery_threshold', 10.0)
        self.declare_parameter('voltage_min', 0.0)
        self.declare_parameter('voltage_max', 14.4)
        
        battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        self.low_threshold = self.get_parameter('low_battery_threshold').get_parameter_value().double_value
        self.voltage_min = self.get_parameter('voltage_min').get_parameter_value().double_value
        self.voltage_max = self.get_parameter('voltage_max').get_parameter_value().double_value
        
        self.battery_sub = self.create_subscription(
            Float64,
            battery_topic,
            self.battery_callback,
            10
        )
        
        self.status_pub = self.create_publisher(
            Point,
            '/battery_status',
            10
        )
        
        self.get_logger().info(f'battery monitor initialized, subscribing to {battery_topic}')
    
    def battery_callback(self, msg):
        voltage = msg.data
        level = (voltage - self.voltage_min) / (self.voltage_max - self.voltage_min)
        level = max(0.0, min(1.0, level))
        low_battery = level < self.low_threshold
        
        # Use Point message: x=voltage, y=level, z=low_battery (0.0 or 1.0)
        status = Point()
        status.x = voltage
        status.y = level
        status.z = 1.0 if low_battery else 0.0
        
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

