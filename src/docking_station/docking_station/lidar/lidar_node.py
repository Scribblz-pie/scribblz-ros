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
    finally:
        processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
