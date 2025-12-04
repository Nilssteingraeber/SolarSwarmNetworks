import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Position: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
