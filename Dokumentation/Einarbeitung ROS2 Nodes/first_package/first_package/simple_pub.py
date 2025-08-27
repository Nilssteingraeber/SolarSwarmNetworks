import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SimplePub(Node):
    def __init__(self):
        super().__init__("simple_pub")
        # Übergibt Namen der Node
        self.publisher_ = self.create_publisher(String, "status", 10)
        # Iniziiert einen Publisher, der Strings auf dem Topic "status" publiziert
        # Nachrichten werden in eine Queue der Länge 10 gelegt
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Iniziiert einen Timer an, der jede Sekunden self.timer_callback ausführt

    def timer_callback(self):
        msg = String()
        msg.data = "Publisher still running..."
        # Iniziiert eine Nachricht
        self.publisher_.publish(msg)
        # Publiziert Nachricht auf dem Topic "status"
        self.get_logger().info("Status publiziert.")
        # Ausgabe in auführendem Terminal
        


def main():
    # rclpy starten
    rclpy.init()

    # Node starten
    simple_pub = SimplePub()
    rclpy.spin(simple_pub)

    # Node zerstören
    simple_pub.destroy_node()

    # rclpy beenden
    rclpy.shutdown()


if __name__ == '__main__':
    main()
