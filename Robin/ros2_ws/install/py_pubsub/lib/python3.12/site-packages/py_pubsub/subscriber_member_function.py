import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription

    def pose_callback(self, msg: Pose):
        self.get_logger().info(
            f"Turtle position -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()