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

from typing import Set
import re, uuid, psutil
from os import cpu_count
from socket import gethostbyname, gethostname


class BaseStatusPub(Node):
    def __init__(self, nid, mac):
        super().__init__('mock_robot_status_pub_%s' % (nid,))
        self.__nid = nid
        self.__mac = mac
        self.__please_override = 'This method of BaseStatusSub does not do anything. Please overwrite it in a child class.'

        self.__publishers = {}
        self.__services = {}
        self.__actions = {}
        self.__allowed_activities = Set() # use add or union to append allowed activities
        self.__activity = None
        self.__timers = {}

        # init services
        self.services['set_activity'] = self.create_service(SetRobotActivity, 'set_robot_activity_%s' % (self.__nid,), self.set_activity_callback) 
        self.services['service_info'] = self.create_service(RobotServiceInfo, 'robot_service_info_%s' % (self.__nid,), self.service_info_callback)
        self.services['interface_info'] = self.create_service(RobotInterfaceInfo, 'robot_interface_info_%s' % (self.__nid,), self.interface_info_callback)
        
        # init publishers
        self.publishers['battery'] = self.create_publisher(RobotBattery, 'robot_battery', 3) # as percent, i.e. 78.46
        self.publishers['cpu'] = self.create_publisher(RobotCpu, 'robot_cpu', 3) # as percentage, i.e. 34.00
        self.publishers['activity'] = self.create_publisher(RobotActivity, 'robot_activity', 3)
        self.publishers['point'] = self.create_publisher(RobotPoint, 'robot_point', 3)
        self.publishers['orientation'] = self.create_publisher(RobotQuaternion, 'robot_orientation', 3)
        self.publishers['misc'] = self.create_publisher(RobotMisc, 'robot_misc', 3)
        self.get_logger().debug('Publishers initialized')
    
    # read-only properties (dict entries can still be changed)
    @property
    def nid(self):
        return self.__nid
    @property
    def mac(self):
        return self.__mac
    @property
    def publishers(self):
        return self.__publishers
    @property
    def services(self):
        return self.__services
    @property
    def actions(self):
        return self.__actions
    @property
    def allowed_activities(self):
        return self.__allowed_activities
    @property
    def timers(self):
        return self.__timers
    
    # read-write properties
    @property
    def activity(self):
        return self.__activity
    @activity.setter
    def activity(self, a):
        if a in self.allowed_activities:
            self.__activity = a

    # service callbacks
    def set_activity_callback(self, request, response):
        raise Exception(self.__please_override)
    def service_info_callback(self, request, response):
        raise Exception(self.__please_override)
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
            return mac.replace(':', '')
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