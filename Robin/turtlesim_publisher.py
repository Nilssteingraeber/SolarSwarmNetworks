import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.get_logger().info('CirclePublisher node has been started')
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.publisher_publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()