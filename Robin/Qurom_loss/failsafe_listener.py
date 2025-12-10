#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FailsafeListener(Node):
    def __init__(self):
        super().__init__('failsafe_listener')
        self.subscription = self.create_subscription(
            String,
            '/swarm/failsafe_cmd',
            self.failsafe_cb,
            10)
        self.get_logger().info('Failsafe Listener started, listening to /swarm/failsafe_cmd')

    def failsafe_cb(self, msg):
        self.get_logger().warn(f'Received failsafe command: {msg.data}')
        # Here you can insert your actual RTB or landing trigger command
        # For now, just log it

def main(args=None):
    rclpy.init(args=args)
    node = FailsafeListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
