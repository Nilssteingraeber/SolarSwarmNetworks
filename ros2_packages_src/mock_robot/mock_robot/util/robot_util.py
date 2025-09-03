import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetRobotActivity
from custom_interfaces.srv import RobotServiceInfo
from custom_interfaces.srv import RobotInterfaceInfo
from custom_interfaces.msg import RobotBattery
from custom_interfaces.msg import RobotCpu
from custom_interfaces.msg import RobotActivity
from custom_interfaces.msg import RobotPoint
from custom_interfaces.msg import RobotQuaternion
from custom_interfaces.msg import RobotMisc
from custom_interfaces.msg import NeighborList

from typing import Set
import re, uuid, psutil
from abc import ABC, abstractmethod
from os import cpu_count
from socket import gethostbyname, gethostname


class BaseStatusPub(ABC, Node):
    def __init__(self, nid, mac):
        super().__init__('mock_robot_status_pub_%s' % (nid,))
        self.__nid = nid
        self.__mac = mac
        self.__please_override = 'This method of BaseStatusSub does not do anything. Please overwrite it in a child class.'

        self.__publisher_dict = {}
        self.__service_dict = {}
        self.__action_dict = {}
        self.__allowed_activities = set() # use add or union to append allowed activities
        self.__activity = ''
        self.__timer_dict = {}
        self.__neighbor_dict = {}

        # init services
        self.__service_dict['set_activity'] = self.create_service(SetRobotActivity, 'set_robot_activity_%s' % (self.__nid,), self.set_activity_callback) 
        self.__service_dict['service_info'] = self.create_service(RobotServiceInfo, 'robot_service_info_%s' % (self.__nid,), self.service_info_callback)
        self.__service_dict['interface_info'] = self.create_service(RobotInterfaceInfo, 'robot_interface_info_%s' % (self.__nid,), self.interface_info_callback)
        
        # init publishers
        self.publisher_dict['battery'] = self.create_publisher(RobotBattery, 'robot_battery', 3) # as percent, i.e. 78.46
        self.publisher_dict['cpu'] = self.create_publisher(RobotCpu, 'robot_cpu', 3) # as percentage, i.e. 34.00
        self.publisher_dict['activity'] = self.create_publisher(RobotActivity, 'robot_activity', 3)
        self.publisher_dict['point'] = self.create_publisher(RobotPoint, 'robot_point', 3)
        self.publisher_dict['orientation'] = self.create_publisher(RobotQuaternion, 'robot_orientation', 3)
        self.publisher_dict['misc'] = self.create_publisher(RobotMisc, 'robot_misc', 3)
        self.publisher_dict['neighbors'] = self.create_publisher(NeighborList, 'neighbors', 3)
        self.get_logger().debug('Publishers initialized')
    
    # read-only properties (dict entries can still be changed)
    @property
    def nid(self):
        return self.__nid
    @property
    def mac(self):
        return self.__mac
    @property
    def publisher_dict(self):
        return self.__publisher_dict
    # @property
    # def service_dict(self):
    #     return self.__service_dict
    @property
    def action_dict(self):
        return self.__action_dict
    @property
    def allowed_activities(self):
        return self.__allowed_activities
    @property
    def timer_dict(self):
        return self.__timer_dict
    @property
    def neighbor_dict(self):
        return self.__neighbor_dict
    
    # read-write properties
    @property
    def activity(self):
        return self.__activity
    @activity.setter
    def activity(self, a):
        if a in self.allowed_activities:
            self.__activity = a

    # service callbacks
    @abstractmethod
    def set_activity_callback(self, request, response):
        raise Exception(self.__please_override)
    @abstractmethod
    def service_info_callback(self, request, response):
        raise Exception(self.__please_override)
    @abstractmethod
    def interface_info_callback(self, request, response):
        raise Exception(self.__please_override)
    
    

# module functions
class Util(object):
    @staticmethod
    def get_mac():
        try:
            mac = ':'.join(re.findall('..', '%012x' % uuid.getnode()))
            return mac
        except Exception as e:
            print('Failed to get MAC:', e)
            return None

    @staticmethod
    def get_nid(mac):
        if type(mac).__name__ == 'str':
            return mac.replace(':', '') # hash(mac)
        else:
            print('Given MAC is not a string')
            return None

    @staticmethod
    def get_battery():
        battery = psutil.sensors_battery()
        if battery != None: # sensors_battery() returns None-object if battery can not be found
            return battery.percent
        else:
            return -1.0

    @staticmethod
    def get_cpu():
        load1, load5, load15 = psutil.getloadavg() # returns 3 values
        return (load1 / cpu_count()) * 100 # average of last minute

    @staticmethod
    def get_ip():
        gethostbyname(gethostname())

    @staticmethod
    def get_neighbors():
        pass # to-do