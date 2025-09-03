import psycopg2
from os import getenv

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from time import time
from abc import ABC, abstractmethod

class BaseStatusSub(ABC, Node):
    def __init__(self):
        super().__init__("robot_status_sub")
        self.__please_override = 'This method of BaseStatusSub does not do anything. Please overwrite it in a child class.'

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

    @property
    def subscription_dict(self):
        return self.__subscription_dict
    @property
    def nodes(self):
        return self.__nodes
    @property
    def conn(self):
        return self.__conn

    def connect_db(self) -> bool:
        try:
            if not self.conn or self.conn.closed:
                self.__conn = psycopg2.connect(
                    dbname = getenv('DB_NAME'),
                    user = getenv('DB_USER'),
                    password = getenv('DB_PASSWORD'),
                    host = getenv('DB_HOST'),
                    port = getenv('DB_PORT'))
                print("Connection to db successful")
            return True
        except Exception as e:
            print("Connection to db failed:", e)
            return False

    @abstractmethod
    def subscription_callback(self, msg):
        raise Exception(self.__please_override)
    @abstractmethod
    def forward_batch(self):
        raise Exception(self.__please_override)
    @abstractmethod
    def batch_timer_callback(self):
        raise Exception(self.__please_override)
    
    def check_nid(self, nid) -> bool:
        # May be used in callback functions to check if a nid exists in nodes.
        # It creates an empty dict in nodes for unfamiliar nids to prevent exceptions when accessing a field.
        # Always returns True. Can be overridden to return False for illegal nids.
        if not nid in self.nodes.keys():
            self.nodes[nid] = {}
        return True
