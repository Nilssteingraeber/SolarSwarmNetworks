import psycopg2
from os import getenv

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from abc import ABC, abstractmethod

class BaseStatusSub(ABC, Node):
    def __init__(self):
        super().__init__("robot_status_sub")

        # parameters
        self.declare_parameter('batch_intervall', 10.0, ParameterDescriptor(description='Time it takes the data sink to forward the next batch.'))
        
        # subscriptions
        self.__subscription_dict = {}
        
        # timers
        BATCH = rclpy.parameter.parameter_value_to_python(
            self.get_parameter('batch_intervall').get_parameter_value())
        self.__batch_timer = self.create_timer(BATCH, self.batch_timer_callback)
        self.get_logger().debug('Timers initialized')
        
        # dict to hold most recent information of each node
        self.__nodes = {}

        # database connection
        self.__conn = None

    # properties
    @property
    def nodes(self):
        return self.__nodes
    
    @property
    def conn(self):
        return self.__conn
    
    
    def getSubscription(self, key):
        if key in self.__subscription_dict.keys():
            return self.__subscription_dict[key]
        return None

    def createSubscription(self, key, cls, topic, callback, queue=10) -> bool:
        if type(key).__name__ != 'str':
            self.get_logger().error('Failed to add subscription: Key must be a string')
        elif key in self.__subscription_dict.keys():
            self.get_logger().error('Failed to add subscription: Key %s already taken', (key,))
        elif queue < 1:
            self.get_logger().error('Failed to add subscription: Queue length too small')
        else:
            try:
                self.__subscription_dict[key] = self.create_subscription(cls, topic, callback, queue)
                return True
            except Exception as e:
                self.get_logger().error('Failed to add subscription: %s', (e,))
        return False


    # methods
    def connect_db(self) -> bool:
        try:
            if not self.conn or self.conn.closed:
                self.__conn = psycopg2.connect(
                    dbname = getenv('DB_NAME'),
                    user = getenv('DB_USER'),
                    password = getenv('DB_PASSWORD'),
                    host = getenv('DB_HOST'),
                    port = getenv('DB_PORT'))
                print('Connection to db successful')
            return True
        except Exception as e:
            print('Connection to db failed:', e)
            try:
                self.get_logger().error('Connection to db failed: %s', (e,))
            except:
                pass
            return False

    def check_nid(self, nid) -> bool:
        # May be used in callback functions to check if a nid exists in nodes.
        # It creates an empty dict in nodes for unfamiliar nids to prevent exceptions when accessing a field.
        # Always returns True. Can be overridden to return False for illegal nids.
        if not nid in self.nodes.keys():
            self.nodes[nid] = {}
        return True
    
    # validator methods
    def validate_point(self, point) -> bool:
        try:
            # x (longitude) should be in range of -180 (west) to 180 (east)
            if point['x'] < -180.0 or point['x'] > 180.0:
                return False
            # y (latitude) should be in range of -90 (south) to 90 (north)
            if point['y'] < -90.0 or point['y'] > 90.0:
                return False
            return True
        except:
            return False

    def validate_quaternion(self, quat) -> bool:
        try:
            sq_sum = 0.0
            # x, y, z, and w should be in range of -1 to 1
            for value in quat.values():
                if value < -1.0 or value > 1.0:
                    return False
                sq_sum += value * value
            # square sum should be 1
            if abs(1.0 - sq_sum) > 0.0001: # sq_sum != 1 not practical
                return False
            return True
        except:
            return False

    def validate_ipv4(self, ip: str) -> bool:
        try:
            num_list = ip.split('.')
            if len(num_list) != 4:
                return False
            for num in num_list:
                if int(num) < 0 or int(num) > 255:
                    return False
            return True
        except:
            self.get_logger().error('Invalid ipv4')
            return False

    def validate_ipv6(self, ip: str) -> bool:
        try:
            col_cnt = ip.count(':')
            if col_cnt < 2 or col_cnt > 7:
                return False
            # check number of segments
            num_list = ip.lower().split(':')
            if len(num_list) < 3 or len(num_list) > 8:
                return False
            # only ::, not : at start or beginning
            if ip[0] == ':' and not ip[1] == ':':
                return False
            if ip[-1] == ':' and not ip[-2] == ':':
                return False
            # check segments
            for num in num_list:
                if len(num) > 4:
                    return False
                for char in num:
                    # chars must be 0 to 9 or a to f
                    if ord(char) not in range(48, 58) and ord(char) not in range(97, 103):
                        return False
            return True
        except:
            self.get_logger().error('Invalid ipv6')
            return False

    def validate_mac(self, mac: str) -> bool:
        try: # only allows : as separator, not -
            if len(mac) != 17:
                return False
            for num in mac.lower().split(':'):
                if len(num) != 2:
                    return False
                for char in num:
                    if ord(char) not in range(48, 58) and ord(char) not in range(97, 103):
                        return False
            return True
        except:
            self.get_logger().error('Invalid mac')
            return False
        
    # override for additional checks
    def validate_neighbor(self, neighbor: str) -> bool:
        if type(neighbor).__name__ != 'str':
            return False
        return self.validate_mac(neighbor)

    def validate_indicator(self, indicator: float) -> bool:
        # possible range 0 to ~120 dBm
        # typical range -30 to -80 dBm
        if type(indicator).__name__ not in ('float', 'int'):
            return False
        return indicator <= 0.0 and indicator >= -130.0
    

    # callbacks
    @abstractmethod
    def subscription_callback(self, msg):
        pass
    
    @abstractmethod
    def forward_batch(self):
        pass
    
    @abstractmethod
    def batch_timer_callback(self):
        pass
    
class Util(object):
    @staticmethod
    def get_nid(mac) -> str:
        # copy from mock_robot.util.robot_util.py
        # changes made to this must also be made there
        if type(mac).__name__ == 'str':
            return mac.replace(':', '') # hash(mac)
        else:
            print('Given MAC is not a string')
            return None