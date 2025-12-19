#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Empty


class StateMachineNode(Node):
    """Broadcasts a simple string state on /robot_state and handles fan ramp for undocking."""

    def __init__(self):
        super().__init__('state_machine')

        self.state = 'docked'

        # State publisher/subscriber
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10,
        )

        # Fan control and undocking start trigger
        self.fan_pub = self.create_publisher(Int32, '/fan_speed', 1)
        self.undocking_start_pub = self.create_publisher(Empty, '/undocking/start', 1)

        # Undocking fan ramp timer state
        self.undocking_ramp_timer = None
        self.undocking_ramp_step = 0  # 0:1300, 1:1600, 2:1800

        # Periodically re-publish current state
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('state machine initialized, starting in docked state')

    def robot_state_callback(self, msg: String):
        """Update state from /robot_state and start undocking fan ramp if needed."""
        new_state = msg.data

        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        self.get_logger().info(f'state changed: {old_state} -> {new_state}')

        if new_state == 'undocking':
            self.start_undocking_fan_ramp()
        else:
            # Cancel any ongoing ramp if we leave undocking
            if self.undocking_ramp_timer is not None:
                self.undocking_ramp_timer.cancel()
                self.undocking_ramp_timer = None

    def start_undocking_fan_ramp(self):
        """Ramp fan: 1300 -> 1600 -> 1800, 3 seconds apart, then trigger undocking start."""
        # Cancel previous ramp if any
        if self.undocking_ramp_timer is not None:
            self.undocking_ramp_timer.cancel()
            self.undocking_ramp_timer = None

        self.undocking_ramp_step = 0

        # Apply first step immediately, then create timer for subsequent steps
        self._apply_undocking_fan_step()
        self.undocking_ramp_timer = self.create_timer(3.0, self._apply_undocking_fan_step)

    def _apply_undocking_fan_step(self):
        """Apply one step of the undocking fan ramp."""
        # If we've left undocking, stop ramping
        if self.state != 'undocking':
            if self.undocking_ramp_timer is not None:
                self.undocking_ramp_timer.cancel()
                self.undocking_ramp_timer = None
            return

        speed = None
        if self.undocking_ramp_step == 0:
            speed = 1300
        elif self.undocking_ramp_step == 1:
            speed = 1500
        elif self.undocking_ramp_step == 2:
            speed = 1700
        else:
            # All steps done; cancel timer
            if self.undocking_ramp_timer is not None:
                self.undocking_ramp_timer.cancel()
                self.undocking_ramp_timer = None
            return

        msg = Int32()
        msg.data = speed
        self.fan_pub.publish(msg)
        self.get_logger().info(f'Undocking fan ramp step {self.undocking_ramp_step + 1}: set fan_speed to {speed}')

        # After final step, trigger undocking start and stop timer
        if self.undocking_ramp_step == 2:
            self.undocking_start_pub.publish(Empty())
            self.get_logger().info('Undocking fan ramp complete, published /undocking/start')
            if self.undocking_ramp_timer is not None:
                self.undocking_ramp_timer.cancel()
                self.undocking_ramp_timer = None

        self.undocking_ramp_step += 1

    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

