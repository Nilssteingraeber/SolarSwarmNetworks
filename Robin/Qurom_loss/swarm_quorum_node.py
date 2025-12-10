#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class SwarmQuorumNode(Node):
    def __init__(self):
        super().__init__('swarm_quorum_node')

        # Parameters
        self.declare_parameter('drone_id', 'drone_1')
        self.declare_parameter('swarm_size', 3)
        self.declare_parameter('heartbeat_topic', '/swarm/heartbeat')
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.declare_parameter('grace_period', 10.0)
        self.declare_parameter('failsafe_action', 'RTL')
        self.declare_parameter('failsafe_topic', '/swarm/failsafe_cmd')

        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.swarm_size = self.get_parameter('swarm_size').get_parameter_value().integer_value
        self.heartbeat_topic = self.get_parameter('heartbeat_topic').get_parameter_value().string_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value
        self.grace_period = self.get_parameter('grace_period').get_parameter_value().double_value
        self.failsafe_action = self.get_parameter('failsafe_action').get_parameter_value().string_value
        self.failsafe_topic = self.get_parameter('failsafe_topic').get_parameter_value().string_value

        # State: last heartbeat timestamps of peers
        self.peer_last_seen = {}
        self.peer_last_seen[self.drone_id] = time.time()

        # Docker Swarm quorum state
        self.docker_quorum_lost = False

        # Publishers and Subscribers
        self.heartbeat_sub = self.create_subscription(String, self.heartbeat_topic, self.heartbeat_cb, 20)
        self.docker_swarm_sub = self.create_subscription(String, '/swarm/docker_swarm_status', self.docker_swarm_status_cb, 10)
        self.failsafe_pub = self.create_publisher(String, self.failsafe_topic, 10)

        # Timer to check quorum regularly
        self.create_timer(1.0, self.check_quorum)

        # Grace timer state
        self.quorum_lost_since = None
        self.failsafe_triggered = False

        self.get_logger().info(f'SwarmQuorumNode started: id={self.drone_id}, swarm_size={self.swarm_size}')

    def heartbeat_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            drone_id = payload.get('drone_id')
            ts = payload.get('ts', time.time())
            if drone_id is None:
                return
        except Exception:
            return
        self.peer_last_seen[drone_id] = ts

    def docker_swarm_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.docker_quorum_lost = payload.get('quorum_lost', False)
        except Exception:
            self.docker_quorum_lost = False

    def get_alive_peer_count(self):
        now = time.time()
        alive = [d for d, t in self.peer_last_seen.items() if (now - t) <= self.heartbeat_timeout]
        return len(alive)

    def quorum_required(self):
        return (self.swarm_size // 2) + 1

    def check_quorum(self):
        now = time.time()
        alive_count = self.get_alive_peer_count()
        required = self.quorum_required()

        # ROS2 heartbeat quorum check
        ros2_quorum_ok = alive_count >= required

        # Overall quorum lost if Docker quorum lost OR ROS2 quorum lost
        quorum_lost = self.docker_quorum_lost or not ros2_quorum_ok

        if not quorum_lost:
            if self.quorum_lost_since is not None:
                self.get_logger().info(f'Quorum restored: alive peers={alive_count}/{self.swarm_size}')
            self.quorum_lost_since = None
            self.failsafe_triggered = False
            return

        # Quorum is lost
        if self.quorum_lost_since is None:
            self.quorum_lost_since = now
            self.get_logger().warn(f'Quorum lost, starting grace period {self.grace_period}s')
            return

        elapsed = now - self.quorum_lost_since
        if elapsed >= self.grace_period and not self.failsafe_triggered:
            self.get_logger().error(f'Quorum lost for {elapsed:.1f}s, triggering failsafe: {self.failsafe_action}')
            self.trigger_failsafe()
            self.failsafe_triggered = True

    def trigger_failsafe(self):
        msg = String()
        msg.data = self.failsafe_action
        self.failsafe_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmQuorumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
