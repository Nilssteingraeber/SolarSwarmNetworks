import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
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

import re, uuid, psutil
from abc import ABC, abstractmethod
from os import cpu_count, popen, getenv
from socket import gethostbyname, gethostname
from random import randint
from mock_robot.util.time_util import TimeUtil

w = getenv('WLANDEV')
if w and w.isalnum():
    WLANDEV = w
else:
    WLANDEV = None

DUMP = getenv('IW_DUMP') # get station dump output for neighbors

class BaseStatusPub(ABC, Node):
    def __init__(self, nid, mac):
        super().__init__('mock_robot_status_pub_%s' % (nid,))
        self.__please_override = 'This method of BaseStatusSub does not do anything. Please overwrite it in a child class.'
        self.__nid = nid
        self.__mac = mac

        self.__publisher_dict = {}
        self.__service_dict = {}
        self.__action_dict = {}
        self.__allowed_activities = set(('Idle', 'Charging', 'MoveToPosition', 'Working')) # use .update() to append allowed activities
        self.__activity = ''
        self.__timer_dict = {}
        self.__neighbor_dict = {}
        
        # init publishers
        self.createPublisher('battery', RobotBattery, 'robot_battery') # 3 by default
        self.createPublisher('cpu', RobotCpu, 'robot_cpu')
        self.createPublisher('activity', RobotActivity, 'robot_activity')
        self.createPublisher('point', RobotPoint, 'robot_point')
        self.createPublisher('orientation', RobotQuaternion, 'robot_orientation')
        self.createPublisher('misc', RobotMisc, 'robot_misc')
        self.createPublisher('neighbors', NeighborList, 'neighbors')
        self.get_logger().debug('Publishers initialized')
        
        # init services
        self.createService('set_activity', SetRobotActivity, 'set_robot_activity_%s' % (self.__nid,), self.set_activity_callback)
        self.createService('service_info', RobotServiceInfo, 'robot_service_info_%s' % (self.__nid,), self.service_info_callback)
        self.createService('interface_info', RobotInterfaceInfo, 'robot_interface_info_%s' % (self.__nid,), self.interface_info_callback)
        
    # getter and create functions
    def getPublisher(self, key):
        if key in self.__publisher_dict.keys():
            return self.__publisher_dict[key]
        return None
    
    def getService(self, key):
        if key in self.__service_dict.keys():
            return self.__service_dict[key]
        return None
    
    def getAction(self, key):
        if key in self.__action_dict.keys():
            return self.__action_dict[key]
        return None
    
    def getTimer(self, key):
        if key in self.__timer_dict.keys():
            return self.__timer_dict[key]
        return None
    
    def createPublisher(self, key, cls, topic, queue=3) -> bool:
        if type(key).__name__ != 'str':
            self.get_logger().error("Failed to add publisher: Key must be a string")
        elif key in self.__publisher_dict.keys():
            self.get_logger().error("Failed to add publisher: Key %s already taken", (key,))
        elif queue < 1:
            self.get_logger().error('Failed to add publisher: Queue length too small')
        else:
            try:
                self.__publisher_dict[key] = self.create_publisher(cls, topic, queue)
                return True
            except Exception as e:
                self.get_logger().error('Failed to add publisher: %s', (e,))
        return False
    
    def createService(self, key, cls, service_name, callback) -> bool:
        if type(key).__name__ != 'str':
            self.get_logger().error('Failed to add service: Key must be a string')
        elif key in self.__service_dict.keys():
            self.get_logger().error('Failed to add service: Key %s already taken', (key,))
        else:
            try:
                self.__service_dict[key] = self.create_service(cls, service_name, callback)
                return True
            except Exception as e:
                self.get_logger().error('Failed to add service: %s', (e,))
        return False
    
    def createAction(self, key, cls, action_name, callback=None) -> bool:
        if type(key).__name__ != 'str':
            self.get_logger().error('Failed to add action: Key must be a string')
        elif key in self.__action_dict.keys():
            self.get_logger().error('Failed to add action: Key %s already taken', (key,))
        else:
            try:
                self.__action_dict[key] = ActionServer(self, 
                cls, action_name, callback)
                return True
            except Exception as e:
                self.get_logger().error('Failed to add action: %s', (e,))
        return False
    
    def createTimer(self, key, sec, callback) -> bool:
        if type(key).__name__ != 'str':
            self.get_logger().error("Failed to add timer: Key must be a string")
        elif key in self.__timer_dict.keys():
            self.get_logger().error('Failed to add timer: Key %s already taken', (key,))
        else:
            try:
                self.__timer_dict[key] = self.create_timer(sec, callback)
                return True
            except Exception as e:
                self.get_logger().error('Failed to add timer: %s', (e,))
        return False
    
    def updateAllowedActivities(self, items) -> bool:
        if type(items).__name__ not in ('list', 'set', 'tuple'):
            self.get_logger().error('Failed to update allowed_activities: Items should be in a list, set, or tuple')
        else:
            for item in items:
                if type(item).__name__ != 'str':
                    self.get_logger().error('Failed to update allowed_activities: Items must all be strings')
                    break
            else: # if all items valid
                try:
                    self.__allowed_activities.update(items)
                    return True
                except Exception as e:
                    self.get_logger().error('Failed to update allowed_activities: %s', (e,))
        return False
    
    # read-only properties (dict entries can still be changed)
    @property
    def nid(self):
        return self.__nid
    
    @property
    def mac(self):
        return self.__mac
    
    @property
    def allowed_activities(self):
        return self.__allowed_activities.copy()
    

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
        return self.__neighbor_dict.copy()
    
    @neighbor_dict.setter
    def neighbor_dict(self, d):
        if type(d).__name__ == 'dict':
            self.__neighbor_dict = d
        elif d == None:
            self.__neighbor_dict.clear()
    

    # service callbacks
    @abstractmethod
    def set_activity_callback(self, request, response):
        pass
    
    @abstractmethod
    def service_info_callback(self, request, response):
        pass
    
    @abstractmethod
    def interface_info_callback(self, request, response):
        pass
    
    # other
    def addHeader(self, msg):
        try:
            msg.header.nid = self.nid
            msg.header.time.sec = TimeUtil.get_timestamp()
            return True
        except:
            return False

    
    

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
        # changes made to this must also be made in sw_robot.util.sw_util
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
            return -1.0 # could not get information

    @staticmethod
    def get_cpu() -> float:
        try:
            load1, load5, load15 = psutil.getloadavg() # returns 3 values
            return (load1 / cpu_count()) * 100 # average of last minute
        except:
            return -1.0 # could not get information

    @staticmethod
    def get_ip() -> str:
        try:
            return gethostbyname(gethostname())
        except:
            return None

    @staticmethod
    def get_neighbors() -> dict:
        neighbors = dict()
        try:
            if DUMP:
                iw_output = popen(f'cat {DUMP}').read()
                print("Got iw_dump.txt")
            else:
                raise Exception("No location for iw_dump.txt was provided")
        except Exception as e:
            print("Failed to get iw_dump.txt:", e)
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
                        except Exception as e:
                            neighbors[mac] = None
                            print(e)
                        break
        return neighbors
