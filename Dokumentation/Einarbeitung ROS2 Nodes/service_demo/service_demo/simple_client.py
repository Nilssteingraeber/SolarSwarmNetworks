import sys
from time import sleep

from rcl_interfaces.msg import ParameterDescriptor
from custom_interfaces.srv import EchoServiceIf
import rclpy
from rclpy.node import Node


class EchoClient(Node):

    def __init__(self):
        super().__init__('echo_client')
        self.cli = self.create_client(EchoServiceIf, 'echo_service_if')
        
        self.declare_parameter('nid', '0', ParameterDescriptor(description='Eine nutzerdefinierte ID der Node'))
        
        self.declare_parameter('msg', 'Default message', ParameterDescriptor(description='Zu schickende Nachricht'))
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = EchoServiceIf.Request()

    def send_request(self):
        self.req.nid = self.get_parameter('nid').get_parameter_value().string_value
        self.req.msg = self.get_parameter('msg').get_parameter_value().string_value
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    echo_client = EchoClient()
    for i in range (0, 11):
        future = echo_client.send_request()
        rclpy.spin_until_future_complete(echo_client, future)
        response = future.result()
        echo_client.get_logger().info('Echo received: %s' % response.msg)
        sleep(2)
    
    echo_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
