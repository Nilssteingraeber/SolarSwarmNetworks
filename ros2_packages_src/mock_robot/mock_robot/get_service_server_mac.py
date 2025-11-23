import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetServiceServerMac
from mock_robot.util.robot_util import Util

### GetServiceServerMac ###
# ---
# string mac

class ServiceServerMacNode(Node):
    def __init__(self):
        self.createService('get_service_server_mac', GetServiceServerMac, 'get_service_server_mac', self.get_service_server_mac_callback)

    def get_service_server_mac_callback(self, request, response):
        mac = Util.get_mac()
        if mac:
            response.mac = mac
        else:
            response.mac = ""
        return response

def main():
    rclpy.init()
    get_service_server_mac_node = ServiceServerMacNode()
    rclpy.spin(get_service_server_mac_node)
    get_service_server_mac_node.destroy_node()
    rclpy.shutdown()
