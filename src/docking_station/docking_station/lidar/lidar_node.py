#!/usr/bin/env python3

import rclpy
from .lidar_processor import LidarProcessor


def main(args=None):
    rclpy.init(args=args)
    processor = LidarProcessor()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()