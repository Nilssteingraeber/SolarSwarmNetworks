import rclpy
from rclpy.node import Node
from custom_interfaces.srv import StringToLowerUpperService

### StringToLowerUpperService ###
# string data
# ---
# string lower
# string upper

class StrToLowerUpperNode(Node):
    def __init__(self):
        self.createService('str_to_lower_upper', StringToLowerUpperService, 'str_to_lower_upper', self.str_to_lower_upper_callback)

    def str_to_lower_upper_callback(self, request, response):
        response.lower = request.data.lower()
        response.upper = request.data.upper()
        return response

def main():
    rclpy.init()
    str_to_lower_upper_node = StrToLowerUpperNode()
    rclpy.spin(str_to_lower_upper_node)
    str_to_lower_upper_node.destroy_node()
    rclpy.shutdown()
