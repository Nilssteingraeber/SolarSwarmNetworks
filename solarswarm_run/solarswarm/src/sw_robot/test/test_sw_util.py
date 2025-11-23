import pytest, rclpy
from sw_robot.util.sw_util import BaseStatusSub
from custom_interfaces.msg import RobotBattery

# https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html

NID = 'test'
MAC = '01:02:03:04:05:06'

class FakeStatusSub(BaseStatusSub):
    def __init__(self):
        BaseStatusSub.__init__(self)
    
    def subscription_callback(self, msg):
        pass

    def forward_batch(self):
        pass
    
    def batch_timer_callback(self):
        pass

@pytest.fixture
def node():
    rclpy.init()
    node = FakeStatusSub()
    yield node
    node.destroy_node()
    rclpy.shutdown()

# getter
def test_check_get_nid(node):
    assert node.check_nid(NID) == True
    node.nodes['test']['test_field'] = 1
    assert node.nodes['test']['test_field'] == 1

def test_get_connection(node):
    assert False # missing test


# connection
def test_connect_db(node):
    assert False # missing test

def test_forward_batch(node):
    assert False


# create and get sub
def test_create_getSubscription(node):
    assert node.createSubscription('test', RobotBattery, 'test_topic', node.subscription_callback)
    assert node.getSubscription('test') != None


# validation
def test_check_nid(node):
    assert False # missing test

def test_validate_point(node):
    assert node.validate_point({'x': 180.0, 'y': 90.0, 'z': 0.0}) == True
    assert node.validate_point({'x': -180.0, 'y': -90.0, 'z': 0.0}) == True
    assert node.validate_point({'x': 0.0, 'y': 91.0, 'z': 0.0}) == False
    assert node.validate_point({'x': 0.0, 'y': -91.0, 'z': 0.0}) == False
    assert node.validate_point({'x': 181.0, 'y': 0.0, 'z': 0.0}) == False
    assert node.validate_point({'x': -181.0, 'y': 0.0, 'z': 0.0}) == False

def test_validate_quaternion(node):
    assert node.validate_quaternion({'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}) == True
    assert node.validate_quaternion({'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0001}) == False
    assert node.validate_quaternion({'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.1}) == False
    assert node.validate_quaternion({'x': 1.0, 'y': 1.0, 'z': 1.0, 'w': 1.0}) == False
    assert node.validate_quaternion({'x': 2.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}) == False

def test_validate_ipv4(node):
    assert node.validate_ipv4('127.0.0.1') == True
    assert node.validate_ipv4('0.0.0.0') == True
    assert node.validate_ipv4('255.255.255.255') == True
    assert node.validate_ipv4('127.0.0.256') == False
    assert node.validate_ipv4('127.0.0.A') == False

def test_validate_ipv6(node):
    assert node.validate_ipv6(':::::::') == True # 7 colons
    assert node.validate_ipv6('::') == True
    assert node.validate_ipv6('::AA:AA::AA') == True
    assert node.validate_ipv6(':') == False
    assert node.validate_ipv6('::::::::') == False # 8 colons
    assert node.validate_ipv6('::AAAAA') == False # more than 4 chars
    assert node.validate_ipv6(':AAAA:') == False

def test_validate_mac(node):
    assert node.validate_mac(MAC) == True
    assert node.validate_mac('00:00:00:00:00:00') == True
    assert node.validate_mac('FF:FF:FF:FF:FF:FF') == True
    assert node.validate_mac('FF:FF:FF:FF:FF') == False
    assert node.validate_mac('FF:FF:FF:FF:FF:FG') == False
    assert node.validate_mac('') == False

def test_validate_neighbor(node):
    assert node.validate_neighbor(MAC)

def test_validate_indicator(node):
    assert node.validate_indicator(1.0) == False
    assert node.validate_indicator(0.0) == True
    assert node.validate_indicator(-130.0) == True
    assert node.validate_indicator(-131.0) == False