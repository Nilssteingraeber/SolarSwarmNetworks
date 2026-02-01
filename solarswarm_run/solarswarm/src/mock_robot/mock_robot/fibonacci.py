import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FibonacciService

### FibonacciService ###
# uint64 target 5
# ---
# uint64[] numbers
# uint64 exact

class FibonacciNode(Node):
    def __init__(self):
        super().__init__('fibonacci')
        self.create_service(FibonacciService, 'fibonacci', self.fibonacci_callback)

    def fibonacci_callback(self, request, response):
        # f0  f1  f2  f3  f4  f5  ...
        #  0   1   1   2   3   5  ...
        target = request.target
        numbers = [0, 1]
        while len(numbers)-1 < target:
            numbers.append(numbers[-1] + numbers[-2]) # last + second last element
        response.numbers = numbers[:target+1] # otherwise f0 would return [0, 1]
        response.exact = numbers[-1] # ftarget is last element
        return response

def main():
    rclpy.init()
    fibonacci_node = FibonacciNode()
    rclpy.spin(fibonacci_node)
    fibonacci_node.destroy_node()
    rclpy.shutdown()
