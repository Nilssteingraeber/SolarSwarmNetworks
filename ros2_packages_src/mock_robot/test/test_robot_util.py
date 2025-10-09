import pytest, rclpy
from mock_robot.util.robot_util import BaseStatusPub, Util
from custom_interfaces.msg import RobotBattery
from custom_interfaces.srv import RobotServiceInfo

# https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html

NID = 'test'
MAC = '01:02:03:04:05:06'

class FakeStatusPub(BaseStatusPub):
    def __init__(self, nid, mac):
        BaseStatusPub.__init__(self, nid, mac)
    
    def set_activity_callback(self, request, response):
        return response
    def service_info_callback(self, request, response):
        return response
    def interface_info_callback(self, request, response):
        return response

@pytest.fixture
def node():
    rclpy.init()
    node = FakeStatusPub(NID, MAC)
    yield node
    node.destroy_node()
    rclpy.shutdown()


# getter
def test_get_nid(node):
    assert node.nid == NID

def test_get_mac(node):
    assert node.mac == MAC

def test_get_allowed_activities(node):
    assert node.allowed_activities == set(('Idle', 'Charging', 'MoveToPosition', 'Working'))

def test_get_activity(node):
    assert node.activity == '' # 'Failed to get activity'

def test_get_neighbor_dict(node):
    assert type(node.neighbor_dict).__name__ == 'dict' # 'Failed to get neighbor_dict'


# setter
def test_set_activity(node):
    # set to allowed activity
    node.activity = 'Idle'
    assert node.activity == 'Idle' # 'Failed to set activity'
    # set to unallowed activity
    node.activity = 'not valid'
    assert node.activity == 'Idle' # 'Set activity to unallowed value'

def test_set_neighbor_dict(node):
    # set to another dict
    d = dict()
    d['1'] = 'a'
    node.neighbor_dict = d
    assert node.neighbor_dict['1'] == 'a' # 'Failed to set neighbor_dict'
    # set to None
    node.neighbor_dict = None
    assert len(node.neighbor_dict) == 0 # 'Failed to clear neighbor_dict'


# create and get pub/service/action/timer
def test_create_getPublisher(node):
    assert node.createPublisher('test', RobotBattery, 'test_topic') # 'Failed to create publisher'
    assert node.getPublisher('test') != None # 'Failed to get publisher'

def test_create_getService(node):
    assert node.createService('test', RobotServiceInfo, 'test_service', lambda request, response: response) # 'Failed to create service'
    assert node.getService('service_info') != None # 'Failed to get service'

def test_create_getAction(node):
    # currently no action in custom_interface
    assert node.createAction('test', int, 'test_action', lambda handle: 1) == False
    assert node.getAction('invalid action') == None # 'Expected None from invalid action'

def test_create_getTimer(node):
    assert node.createTimer('test', 5, lambda: 1) == True
    assert node.getTimer('test') != None


# update allowed_activities
def test_updateAllowedActivities(node):
    # with invalid type
    assert node.updateAllowedActivities(1) == False # 'Expected False for invalid type'
    # with invalid type of items
    assert node.updateAllowedActivities(('1', 1)) == False # 'Expected False for invalid item type'
    # with valid input
    assert node.updateAllowedActivities(('Testing',)) == True # 'Failed to update allowed_activities'


# other
def test_addHeader(node):
    msg = RobotBattery()
    assert node.addHeader(msg) == True # 'Failed to add header to msg'


# Util
def test_get_mac():
    assert Util.get_nid(MAC) != None # 'Failed to get nid from mac'
    assert Util.get_nid(1) == None # 'Expected None from invalid mac'

def test_get_battery():
    battery = Util.get_battery()
    assert battery == -1.0 or (battery >= 0.0 and battery <= 100.0)

def test_get_cpu():
    cpu = Util.get_cpu()
    assert cpu == -1.0 or (cpu >= 0.0 and cpu <= 100.0)

def test_get_ip():
    ip = Util.get_ip()
    assert ip == None or (type(ip).__name__ == 'str' and len(ip) in range(7, 16))

def test_get_neighbors():
    neighbors = Util.get_neighbors()
    assert neighbors == None or type(neighbors).__name__ == 'dict'