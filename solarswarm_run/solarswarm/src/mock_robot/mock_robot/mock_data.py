import rclpy
from mock_robot.util.robot_util import BaseStatusPub, Util
from mock_robot.util.time_util import TimeUtil
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
from custom_interfaces.msg import NeighborList

from custom_interfaces.srv import StrToLowerUpperService

from typing import Set
from datetime import datetime
from numpy import array
from random import randint
from os import popen
from json import dumps, loads
from mock_robot.util.mock_position import MockPosition

import logging

logging.basicConfig(
    filename="/home/ubuntu/logs/robot.log",
    level=logging.INFO,
    format="[mock_data] %(levelname)s: %(message)s (%(asctime)s)",
) # formatting options: https://docs.python.org/3/library/logging.html#logrecord-attributes
# usage: logging.info(<message>), logging.warning(<message>), ...

class MockRobotStatusPub(BaseStatusPub, MockPosition):
    def __init__(self, nid, mac):
        # assign node name
        BaseStatusPub.__init__(self, nid, mac)
        MockPosition.__init__(self)

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
        self.createTimer('system', SYSTEM, self.system_timer_callback)
        self.createTimer('geo', GEO, self.geo_timer_callback)
        self.createTimer('misc', MISC, self.misc_timer_callback)
        self.get_logger().debug('Timers initialized')
        
        # robot activity
        # self.allowed_activities.update({'Idle', 'Charging', 'MoveToPosition', 'Working'}) # standard activities/states since 0.11.0
        print(self.allowed_activities)
        self.activity = 'Working'
        print('Initial activity:', self.activity)
        self.get_logger().debug('Activity set')


        self.create_service(StrToLowerUpperService, 'str_to_lower_upper_%s' % (nid,), self.str_to_lower_upper_callback)


    def str_to_lower_upper_callback(self, request, response):
        response.lower = request.data.lower()
        response.upper = request.data.upper()
        return response


    # service callbacks
    def set_activity_callback(self, request, response): 
        if request.activity in self.allowed_activities:
            match request.activity:
                case 'MoveToPosition':    
                    try:
                        print(request.details, type(request.details))
                        point = loads(request.details) # details must contain a json string with at least valid x and y coordinates 
                        self.points = [array([float(point['x']), float(point['y'])])]
                        self.goal = 0
                    except Exception as e:
                        self.get_logger().error('Failed to parse JSON string %s' % (e,))
                        response.msg = ' Details could not be translated into a point: The field should contain a json string with at least keys x and y and floats as their values.'
                        return response
                case 'Working':
                    try:
                        route = int(request.details)
                        if route in range(len(self.mock_routes)):
                            self.points == self.mock_routes[route]
                            self.get_logger().info('Route %d selected' % (route,))
                    except:
                        route = randint(0, len(self.mock_routes)-1)
                        self.points == self.mock_routes[route] # if no valid route, pick random
                        self.get_logger().info('Failed to set points to desired mock route. Route %d was chosen instead.' % (route,))
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
            interface_list = list(map(lambda entry: entry.lstrip(), interface_list))
            if request.interface and request.interface in interface_list:
                response.definition = popen('ros2 interface show %s' % (request.interface,)).read()
                response.interfaces = []
            else:
                response.definition = ''
                response.interfaces = interface_list
        except Exception as e:
                response.definition = ''
                response.interfaces = []
                self.get_logger().error('Error in interface_info_callback: %s' % (e,))
        return response
    
    
    # timer callbacks
    def system_timer_callback(self):
        # battery
        msg = RobotBattery()
        msg.data = Util.get_battery()
        if self.addHeader(msg):
            self.getPublisher('battery').publish(msg)
        
        # cpu
        msg = RobotCpu()
        msg.data = Util.get_cpu()
        if self.addHeader(msg):
            self.getPublisher('cpu').publish(msg)

        # activity
        msg = RobotActivity()
        msg.activity = self.activity
        if self.addHeader(msg):
            self.getPublisher('activity').publish(msg)
    
    def geo_timer_callback(self):
        # point
        # simulate movement towards coordinate goal
        if not self.activity in ('Idle', 'Charging'):
            if self.advance_position(): # True if point is reached
                if self.activity == 'Working':
                    self.goal = (self.goal+1) % len(self.points) # determine next goal (cylce back to first)
                elif self.activity == 'MoveToPoint': # wait after 'MoveToPoint'
                    self.activity = 'Idle'
        msg = RobotPoint()
        msg.x = float(self.current[0]) # beware: arrays contain numpy.float objects, but msg requires normal float
        msg.y = float(self.current[1])
        msg.z = 0.0
        if self.addHeader(msg):
            self.getPublisher('point').publish(msg)
        
        # orientation
        msg = RobotQuaternion()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.w = 1.0
        if self.addHeader(msg):
            self.getPublisher('orientation').publish(msg)

        # neighbors
        neighbors = []
        indicators = []
        msg = NeighborList()
        self.neighbor_dict = Util.get_neighbors()
        for item in self.neighbor_dict.items():
            neighbors.append(item[0])
            indicators.append(item[1])
        msg.neighbors = neighbors
        msg.indicators = indicators
        if self.addHeader(msg):
            self.getPublisher('neighbors').publish(msg)
    
    def misc_timer_callback(self):
        # datetime using format 'year-month-day hour:min:sec.ms'
        # data["datetime"] = str(datetime.now())
        
        msg = RobotMisc()
        msg.ipv4 = Util.get_ip()
        msg.mac = self.mac # Util.get_mac()
        if self.addHeader(msg):
            self.getPublisher('misc').publish(msg)



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
