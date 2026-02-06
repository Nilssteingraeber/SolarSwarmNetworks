import rclpy
from rclpy.node import Node
from custom_interfaces.srv import CommandToPositionService

### CommandToPositionService ###
# string lat ""
# string lon ""
# string alt ""
# ---
# string msg


class CommandToPositionServiceNode(Node):
    def __init__(self):
        super().__init__('command_to_position_service')
        self.create_service(CommandToPositionService, 'command_to_position_service', self.command_to_position_service_callback)

    def command_to_position_service_callback(self, request, response):
        response.msg = "NOT IMPLEMENTED YET"
        return response

def main():
    rclpy.init()
    command_to_position_service_node = CommandToPositionServiceNode()
    rclpy.spin(command_to_position_service_node)
    command_to_position_service_node.destroy_node()
    rclpy.shutdown()
