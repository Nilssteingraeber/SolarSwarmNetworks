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
from os import cpu_count, popen, getenv
from socket import gethostbyname, gethostname
from random import randint

if getenv('WLANDEV').isalnum():
    WLANDEV = getenv('WLANDEV')
else:
    WLANDEV = None

class BaseStatusPub(ABC, Node):
    def __init__(self, nid, mac):
        super().__init__('mock_robot_status_pub_%s' % (nid,))
        self.__nid = nid
        self.__mac = mac
        self.__please_override = 'This method of BaseStatusSub does not do anything. Please overwrite it in a child class.'

        self.__publisher_dict = {}
        self.__service_dict = {}
        self.__action_dict = {}
        self.__allowed_activities = set(('Idle', 'Charging', 'MoveToPosition', 'Working')) # use .update() to append allowed activities
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
    
    # read-write properties
    @property
    def activity(self):
        return self.__activity
    @activity.setter
    def activity(self, a):
        if a in self.__allowed_activities:
            self.__activity = a
        else:
            self.get_logger().warning('Failed to set activity')
    @property
    def neighbor_dict(self):
        return self.__neighbor_dict
    @neighbor_dict.setter
    def neighbor_dict(self, d):
        if type(d).__name__ == 'dict':
            self.__neighbor_dict = d
        elif d == None:
            self.__neighbor_dict.clear()

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
    def get_mac() -> str:
        try:
            mac = ':'.join(re.findall('..', '%012x' % uuid.getnode()))
            return mac
        except Exception as e:
            print('Failed to get MAC:', e)
            return None

    @staticmethod
    def get_nid(mac) -> str:
        if type(mac).__name__ == 'str':
            return mac.replace(':', '') # hash(mac)
        else:
            print('Given MAC is not a string')
            return None

    @staticmethod
    def get_battery() -> float:
        battery = psutil.sensors_battery()
        if battery != None: # sensors_battery() returns None-object if battery can not be found
            return battery.percent
        else:
            return -1.0

    @staticmethod
    def get_cpu() -> float:
        load1, load5, load15 = psutil.getloadavg() # returns 3 values
        return (load1 / cpu_count()) * 100 # average of last minute

    @staticmethod
    def get_ip() -> str:
        gethostbyname(gethostname())

    @staticmethod
    def get_neighbors() -> dict: # to-do: replace with actual function
        neighbors = dict()
        try:
            if not WLANDEV:
                raise ValueError('No legal WLAN device was given. String must not have special characters nor be empty.')
            iw_output = popen(f'sudo iw dev {WLANDEV} station dump').read()
            # iw_output = 'Station 38:00:25:52:f7:10 (on wlp0s20f3)\n\tinactive time:\t8 ms\n\trx bytes:\t783842\n\trx packets:\t16667\n\ttx bytes:\t1364\n\ttx packets:\t10\n\ttx retries:\t3\n\ttx failed:\t0\n\trx drop misc:\t0\n\tsignal:  \t-30 [-30, -32] dBm\n\tsignal avg:\t-29 [-29, -31] dBm\n\ttx duration:\t0 us\n\trx bitrate:\t1.0 MBit/s\n\trx duration:\t0 us\n\tauthorized:\tyes\n\tauthenticated:\tyes\n\tassociated:\tyes\n\tpreamble:\tlong\n\tWMM/WME:\tyes\n\tMFP:\t\tno\n\tTDLS peer:\tno\n\tDTIM period:\t0\n\tbeacon interval:100\n\tconnected time:\t1791 seconds\n\tassociated at [boottime]:\t783.050s\n\tassociated at:\t1758713521111 ms\n\tcurrent time:\t1758715312271 ms\n'
        except Exception as e:
            print(e)
            return None
        
        stations = iw_output.split('Station ')
        for station in stations:
            if station and len(station) > 17:
                mac = station[0:17]
                lines = station[17:].split('\n\t')
                for line in lines:
                    if 'signal avg' in line:
                        signal_avg = line.partition(':')
                        try:
                            num = signal_avg[2].lstrip().replace('\t', '').partition(' ')[0] # remove spaces, remove \t, get first value
                            neighbors[mac] = float(num) # parse float
                        except:
                            neighbors[mac] = None
                        break
        return neighbors
