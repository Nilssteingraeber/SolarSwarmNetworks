import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import time

class DockerSwarmQuorumChecker(Node):
    def __init__(self):
        super().__init__('docker_swarm_quorum_checker')
        self.pub = self.create_publisher(String, '/swarm/docker_swarm_status', 10)
        self.timer = self.create_timer(5.0, self.check_quorum)

    def check_quorum(self):
        try:
            output = subprocess.check_output(['docker', 'node', 'ls', '--format', '{{.ID}} {{.Status}} {{.ManagerStatus}}'])
            lines = output.decode('utf-8').strip().split('\n')
            managers = [l for l in lines if 'Leader' in l or 'Reachable' in l]
            alive_count = len(managers)
            total_managers = len(lines)

            quorum_lost = alive_count < ((total_managers // 2) + 1)

            status = {
                'timestamp': time.time(),
                'managers_alive': alive_count,
                'managers_total': total_managers,
                'quorum_lost': quorum_lost
            }

            msg = String()
            msg.data = json.dumps(status)
            self.pub.publish(msg)

            if quorum_lost:
                self.get_logger().warn(f'Quorum lost! Managers alive: {alive_count}/{total_managers}')
            else:
                self.get_logger().info(f'Quorum OK: {alive_count}/{total_managers}')

        except Exception as e:
            self.get_logger().error(f'Error checking Docker swarm status: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DockerSwarmQuorumChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
