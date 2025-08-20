import rclpy
# from example_interfaces.msg import String
# from example_interfaces.msg import Float64
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion
from custom_interfaces.msg import RobotBattery
from custom_interfaces.msg import RobotCpu
from custom_interfaces.msg import RobotActivity
from custom_interfaces.msg import RobotPoint
from custom_interfaces.msg import RobotQuaternion
from custom_interfaces.msg import RobotMisc

from json import loads
from datetime import datetime
from time import time, sleep
from sw_util import connect_db, BaseStatusSub
from psycopg2 import Error


DB_TABLE_NAME = 'Roboter' # os.getenv('DB_TABLE_NAME')
DB_COLUMN_NAMES = { # <local name>:<name in db>
    'battery': '', # os.getenv('DB_COLUMN_NAME_BATTERY')
    'cpu': '', # os.getenv('DB_COLUMN_NAME_CPU')
    'activity': '', # os.getenv('DB_COLUMN_NAME_ACTIVITY')
    'point': '', # os.getenv('DB_COLUMN_NAME_POINT')
    'orientation': '', # os.getenv('DB_COLUMN_ORIENTATION')
    'misc': '', # os.getenv('DB_COLUMN_NAME_MISC')
}

class RobotStatusSub(BaseStatusSub):
    def __init__(self):
        super().__init__()
        # parameters
        
        # subscriptions
        self.subscriptions['battery'] = self.create_subscription(RobotBattery, 'robot_battery', self.subscription_callback, 3)
        self.subscriptions['cpu'] = self.create_subscription(RobotCpu, 'robot_cpu', self.subscription_callback, 3)
        self.subscriptions['activity'] = self.create_subscription(RobotActivity, 'robot_activity', self.subscription_callback, 3)
        self.subscriptions['point'] = self.create_subscription(RobotPoint, 'robot_point', self.subscription_callback, 3)
        self.subscriptions['orientation'] = self.create_subscription(RobotQuaternion, 'robot_orientation', self.subscription_callback, 3)
        self.subscriptions['misc'] = self.create_subscription(RobotMisc, 'robot_misc', self.subscription_callback, 3)
        self.get_logger().debug('Subscriptions initialized')

    def forward_batch_test(self):
        for node in self.nodes.keys():
            print(f'nid: {node}\t battery: {self.nodes[node]["battery"]}%\t cpu: {self.nodes[node]["cpu"]}%',
                f'activity: {self.nodes[node]["activity"]}',
                f'point: {self.nodes[node]["point"]}',
                f'orientation: {self.nodes[node]["orientation"]}',
                f'misc: {self.nodes[node]["misc"]}', sep='\n\t', end='\n\n')
    
    def forward_batch(self):
        tries = 5
        while tries != 0:
            if self.connect_db():
                try:
                    t = time()
                    with self.conn.cursor() as cursor:
                        for node in self.nodes.keys():
                            if t - self.nodes[node]['last'] < 30.0:
                                cursor.execute("""INSERT INTO %s (%s, %s, %s, %s, %s, %s)
                                            VALUES (%f, %f, %s, %s, %s, %s)""" % (
                                                # table name
                                                DB_TABLE_NAME,
                                                # column names
                                                DB_COLUMN_NAMES['battery'],
                                                DB_COLUMN_NAMES['cpu'],
                                                DB_COLUMN_NAMES['activity'],
                                                DB_COLUMN_NAMES['point'],
                                                DB_COLUMN_NAMES['orientation'],
                                                DB_COLUMN_NAMES['misc'],
                                                # values
                                                self.nodes[node]['battery'],
                                                self.nodes[node]['cpu'],
                                                self.nodes[node]['activity'],
                                                self.nodes[node]['point'],
                                                self.nodes[node]['orientation'],
                                                self.nodes[node]['misc'],
                                            ))
                        self.conn.commit()
                    print('%s: Batch forwarded' % (str(datetime.now()),))
                except Error as e: # psycopg2.Error
                    self.get_logger().error('SQL error: %s' % (e,))
                    try:
                        self.conn.rollback()
                    except:
                        self.get_logger().error('Error: Rollback failed')
                except Exception as e:
                    self.get_logger().error('DB connection failed: %s' % (e,))
            else:
                tries -= 1
                sleep(1)

    # timer callbacks
    def batch_timer_callback(self):
        self.forward_batch_test()
        # self.forward_batch()
    
    # subscription callbacks
    def subscription_callback(self, msg):
        if self.check_nid(msg.nid):
            match type(msg).__name__:
                case 'RobotBattery':
                    self.nodes[msg.nid]['battery'] = msg.data
                case 'RobotCpu':
                    self.nodes[msg.nid]['cpu'] = msg.data
                case 'RobotActivity':
                    self.nodes[msg.nid]['activity'] = msg.activity
                case 'RobotPoint':
                    self.nodes[msg.nid]['point'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}
                case 'RobotQuaternion':
                    self.nodes[msg.nid]['orientation'] = {'x': msg.x, 'y': msg.y, 'z': msg.z, 'w': msg.w}
                case 'RobotMisc':
                    try:
                        data = loads(msg.data)
                        self.nodes[msg.nid]['misc'] = data
                    except Exception as e:
                        self.get_logger().error('Failed to parse JSON: %s' % (e,))
                case _:
                    self.get_logger().error('Missing match case for class %s' % (type(msg).__name__))


def main():
    # rclpy starten
    rclpy.init()

    # Node starten
    robot_status_sub = RobotStatusSub()
    rclpy.spin(robot_status_sub)

    if robot_status_sub.conn:
        try:
            robot_status_sub.conn.close()
        except Exception as e:
            print('Failed to close DB connection:', e)
    
    # Node zerstören
    robot_status_sub.destroy_node()
    
    # rclpy beenden
    rclpy.shutdown()


if __name__ == "__main__":
    main()
