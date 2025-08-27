from custom_interfaces.srv import EchoServiceIf

import rclpy
from rclpy.node import Node


class EchoService(Node):

    def __init__(self):
        super().__init__('echo_service')
        self.srv = self.create_service(EchoServiceIf, 'echo_service_if', self.echo_callback)

    def echo_callback(self, request, response):
        response.msg = request.msg
        self.get_logger().info('Received from %s:\n%s\n' % (request.nid, request.msg))

        return response


def main():
    rclpy.init()

    echo_service = EchoService()

    rclpy.spin(echo_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
