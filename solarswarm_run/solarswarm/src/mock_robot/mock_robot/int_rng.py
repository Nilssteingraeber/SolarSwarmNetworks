import rclpy
from rclpy.node import Node
from custom_interfaces.srv import IntRngService
from random import randint

### IntRngService ###
# uint64 max 100
# uint64 min 0
# ---
# uint64 number

class IntRngNode(Node):
    def __init__(self):
        super().__init__('int_rng')
        self.create_service(IntRngService, 'int_rng', self.int_rng_callback)

    def int_rng_callback(self, request, response):
        response.number = randint(request.min, request.max)
        return response

def main():
    rclpy.init()
    int_rng_node = IntRngNode()
    rclpy.spin(int_rng_node)
    int_rng_node.destroy_node()
    rclpy.shutdown()
