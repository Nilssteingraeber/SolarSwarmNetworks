import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SimpleSub(Node):
    def __init__(self):
        super().__init__("simple_sub")
        # Übergibt Namen der Node
        self.subscription = self.create_subscription(
            String,
            "status",
            self.listener_callback,
            10)
        # Iniziiert einen Subscriber, der Strings auf dem Topic "status" abhört
        # Nachrichten werden in eine Queue der Länge 10 gelegt
        # Bei Empfang einer Nachricht, wird self.listener_callback ausgeführt

    def listener_callback(self, msg):
        self.get_logger().info("Message received: " + msg.data)


def main():
    # rclpy starten
    rclpy.init()

    # Node starten
    simple_sub = SimpleSub()
    rclpy.spin(simple_sub)
    
    # Node zerstören
    simple_sub.destroy_node()
    
    # rclpy beenden
    rclpy.shutdown()


if __name__ == "__main__":
    main()