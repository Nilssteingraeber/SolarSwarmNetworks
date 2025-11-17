import rclpy
from rclpy.node import Node
from custom_interfaces.srv import IntAvgService

### IntAvgService ###
# int64[] ints
# ---
# int64 avg

class IntAvgNode(Node):
    def __init__(self):
        self.createService('int_avg', IntAvgService, 'int_avg', self.int_avg_callback)

    def int_avg_callback(self, request, response):
        response.avg = round(sum(request.ints)/len(request.ints), 2) # round to 2 decimal places
        return response

def main():
    rclpy.init()
    int_avg_node = IntAvgNode()
    rclpy.spin(int_avg_node)
    int_avg_node.destroy_node()
    rclpy.shutdown()
