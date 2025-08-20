import rclpy
from robot_util import BaseStatusPub
from rcl_interfaces.msg import ParameterDescriptor
# from example_interfaces.msg import String
# from example_interfaces.msg import Float64
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion
# from custom_interfaces.srv import SetRobotActivity
# from custom_interfaces.srv import RobotServiceInfo
# from custom_interfaces.srv import RobotInterfaceInfo
from custom_interfaces.msg import RobotBattery
from custom_interfaces.msg import RobotCpu
from custom_interfaces.msg import RobotActivity
from custom_interfaces.msg import RobotPoint
from custom_interfaces.msg import RobotQuaternion
from custom_interfaces.msg import RobotMisc

from typing import Set
from datetime import datetime
from numpy import array
from random import randint
from os import popen
from json import dumps
from robot_util import Util
from mock_position import MockPosition


class MockRobotStatusPub(BaseStatusPub, MockPosition):
    def __init__(self, nid, mac):
        # assign node name
        super().__init__(nid, mac)

        # declare parameters
        self.declare_parameter('system_intervall', 3.0, ParameterDescriptor(description='Time it takes for battery, cpu, and activity to be published again.'))
        self.declare_parameter('geo_intervall', 1.0, ParameterDescriptor(description='Time it takes for point and orientation to be published again.'))
        self.declare_parameter('misc_intervall', 20.0, ParameterDescriptor(description='Time it takes for a json string to be published again.'))
        self.get_logger().debug('Parameters declared')
        
        # timer intervalls
        SYSTEM = rclpy.parameter.parameter_value_to_python(
            self.get_parameter('system_intervall').get_parameter_value())
        GEO = rclpy.parameter.parameter_value_to_python(
            self.get_parameter('geo_intervall').get_parameter_value())
        MISC = rclpy.parameter.parameter_value_to_python(
            self.get_parameter('misc_intervall').get_parameter_value())
        
        # init timers
        self.timers['system'] = self.create_timer(SYSTEM, self.system_timer_callback)
        self.timers['geo'] = self.create_timer(GEO, self.geo_timer_callback)
        self.timers['misc'] = self.create_timer(MISC, self.misc_timer_callback)
        self.get_logger().debug('Timers initialized')
        
        # robot activity
        self.allowed_activities.union(Set(('auto', 'idle', 'manual', 'recharge')))
        self.activity = 'auto'
        self.get_logger().debug('Activity set')
    

    # service callbacks
    def set_activity_callback(self, request, response): 
        if request.activity in self.allowed_activities:
            match request.activity:
                case 'manual':    
                    try:
                        point = dumps(request.details) # details must contain a json string with at least valid x and y coordinates 
                        self.points = [array([point['x'], point['y']])]
                        self.goal = 0
                    except Exception as e:
                        self.get_logger().error('Failed to parse JSON string')
                        response.msg = e + ' Details could not be translated into a point: The field should contain a json string with at least keys x and y and floats as their values.'
                        return response
                case 'auto':
                    try:
                        route = int(request.details)
                        if route in range(len(self.mock_routes)):
                            self.points == self.__mock_routes[route]
                    except:
                        route_variant = randint(0, len(self.mock_routes)-1)
                        self.points == self.mock_routes[route_variant] # if no valid route, pick random
                        self.get_logger().debug('Failed to set points to desired mock route. Route %d was chosen instead.', (route_variant,))
                case _:
                    pass
            self.activity = request.activity
            response.msg = 'Activity updated'
            self.get_logger().info('Activity updated to: %s' % (request.activity,))
        else:
            response.msg = 'Undefined activity. Set of defined activities: %s' % (str(self.allowed_activities),)
        return response
    
    def service_info_callback(self, request, response):
        try:
            if not request.service: # empty field service
                services_list = popen('ros2 service list -t').read().splitlines() # get list of services and respective types
                response.services = services_list
                response.msg = '%d services found' % (len(services_list),)
            else: # filter using field service
                if request.service.replace('_', '').replace('/', '').isalnum(): # check for legal input
                    services_list = popen('ros2 service list -t | grep %s' % (request.service,)).read().splitlines()
                    response.services = services_list
                    response.msg = '%d filtered services found' % (len(services_list),)
                else:
                    response.msg = 'Service failed: Field service contained illegal characters (must be alphanumeric or _)'
                    response.services = []
        except Exception as e:
                response.msg = 'Service failed: An error has occurred'
                response.services = []
                print('Error in service_info_callback:', e)
        return response
    
    def interface_info_callback(self, request, response): # returns a) list of all interfaces (can be empty) and empty definition or b) an interface definition and empty list
        try:
            interface_list = popen('ros2 interface list').read().splitlines()
            if request.interface and request.interface in interface_list:
                response.definition = popen('ros2 interface show').read()
                response.interfaces = []
            else:
                response.definition = ''
                response.interfaces = interface_list
        except Exception as e:
                response.msg = 'Service failed: An error has occurred'
                print('Error in interface_info_callback:', e)
        return response
    
    
    # timer callbacks
    def system_timer_callback(self):
        # battery
        msg = RobotBattery()
        msg.nid = self.nid
        msg.data = Util.get_battery()
        self.battery_pub.publish(msg)
        
        # cpu
        msg = RobotCpu()
        msg.nid = self.nid
        msg.data = Util.get_cpu()
        self.cpu_pub.publish(msg)

        # activity
        msg = RobotActivity()
        msg.nid = self.nid
        msg.activity = self.activity
        self.activity_pub.publish(msg)
    
    def geo_timer_callback(self):
        # point
        # simulate movement towards coordinate goal
        if not self.activity in ('idle', 'recharge'):
            if self.advance_position(): # True if point is reached
                if self.activity == 'auto':
                    self.goal = (self.goal+1) % len(self.points) # determine next goal (cylce back to first)
                else: # wait when in 'manual'
                    self.activity = 'idle'
        msg = RobotPoint()
        msg.nid = self.nid
        msg.x = float(self.current[0]) # arrays contain numpy.float objects, but geometry_messages requires float
        msg.y = float(self.current[1])
        msg.z = 0.0
        self.point_pub.publish(msg)
        
        # orientation
        msg = RobotQuaternion()
        msg.nid = self.nid
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.w = 1.0
        self.orientation_pub.publish(msg)
    
    def misc_timer_callback(self):
        data = {}
        
        # MAC
        data["mac"] = self.mac
        
        # IP
        data["ip"] = Util.get_ip()
        
        # datetime using format 'year-month-day hour:min:sec.ms'
        data["datetime"] = str(datetime.now())
        
        msg = RobotMisc()
        msg.nid = self.nid
        msg.data = dumps(data)
        self.misc_pub.publish(msg)



def main():
    # start up
    rclpy.init()
    
    # get mac
    mac = Util.get_mac()
    if mac: # if not None, proceed
        mock_robot_status_pub = MockRobotStatusPub(Util.get_nid(mac), mac)
        print("Node ready")

        # start up
        rclpy.spin(mock_robot_status_pub)
        mock_robot_status_pub.get_logger().info('Mock-Robot ready using nid %s' % (str(mock_robot_status_pub.nid),))
        mock_robot_status_pub.get_logger().info('Mock-Robot running')
        
        # shut down
        mock_robot_status_pub.destroy_node()
        mock_robot_status_pub.get_logger().info('Mock-Robot destroyed')
    
    rclpy.shutdown()