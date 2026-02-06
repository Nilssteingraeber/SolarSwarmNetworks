import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import sys
import time
from datetime import datetime

# Relative Import für die Datenbank anpassen
sys.path.append('/home/vboxuser/Documents/SoftwarePrakt/SolarSwarm-Node-Visualizer/backend')
from database import SessionLocal
from models import TurtlePose

class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)
        self.last_save_time = time.time()
        self.save_interval = 30  # 30 Sekunden Intervall
        self.latest_pose = None
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        current_time = time.time()
        
        # Immer die neueste Pose speichern
        self.latest_pose = msg
        
        # Nur speichern wenn das Intervall abgelaufen ist
        if current_time - self.last_save_time >= self.save_interval:
            self.save_to_database()
            self.last_save_time = current_time
            
            # Loggen dass wir gespeichert haben
            self.get_logger().info(
                f'Position gespeichert: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}'
            )
    
    def save_to_database(self):
        if self.latest_pose is None:
            return
            
        msg = self.latest_pose
        db = SessionLocal()
        try:
            db_pose = TurtlePose(
                x=msg.x,
                y=msg.y,
                theta=msg.theta,
                linear_velocity=msg.linear_velocity,
                angular_velocity=msg.angular_velocity,
                timestamp=datetime.now()
            )
            db.add(db_pose)
            db.commit()
        except Exception as e:
            self.get_logger().error(f"Database error: {str(e)}")
            db.rollback()
        finally:
            db.close()

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()