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
from custom_interfaces.msg import NeighborList

from json import loads
from time import sleep
from sw_robot.util.time_util import TimeUtil
from sw_robot.util.sw_util import BaseStatusSub
from psycopg2 import Error
from os import getenv

DB_TABLE_NAMES = {
    'robot': getenv('DB_TABLE_NAME_ROBOT'),
    'neighbor': getenv('DB_TABLE_NAME_NEIGHBOR'),
    'status': getenv('DB_TABLE_NAME_STATUS'),
}

DB_COLUMN_NAMES = {
    # ROBOT
    'robot_id': getenv('DB_COLUMN_NAME_ROBOT_ID'),
    'nid': getenv('DB_COLUMN_NAME_NID'),
    'ipv4': getenv('DB_COLUMN_NAME_IPV4'),
    'ipv6': getenv('DB_COLUMN_NAME_IPV6'),
    'mac': getenv('DB_COLUMN_NAME_MAC'),
    # STATE
    'state_id': getenv('DB_COLUMN_NAME_STATE_ID'),
    'state': getenv('DB_COLUMN_NAME_STATE_'),
    # STATUS
    'status_id': getenv('DB_COLUMN_NAME_STATUS_ID'),
    'battery': getenv('DB_COLUMN_NAME_BATTERY'),
    'cpu': getenv('DB_COLUMN_NAME_CPU_1'),
    'point': getenv('DB_COLUMN_NAME_POINT'),
    'orientation': getenv('DB_COLUMN_NAME_ORIENTATION'),
    'last_heard': getenv('DB_COLUMN_NAME_LAST_HEARD'),
    # 'misc': getenv('DB_COLUMN_NAME_MISC'),
    # NEIGHBOR
    'neighbor': getenv('SB_COLUMN_NAME_NEIGHBOR'),
    'strength': getenv('DB_COLUMN_NAME_STRENGTH'),
}

class RobotStatusSub(BaseStatusSub):
    def __init__(self):
        super().__init__()
        self.__nid_map = dict() # <nid>:<robot_id>
        self.__state_map = dict() # <activity>:<state_id>
        
        # subscriptions
        self.subscription_dict['battery'] = self.create_subscription(RobotBattery, 'robot_battery', self.subscription_callback, 10)
        self.subscription_dict['cpu'] = self.create_subscription(RobotCpu, 'robot_cpu', self.subscription_callback, 10)
        self.subscription_dict['activity'] = self.create_subscription(RobotActivity, 'robot_activity', self.subscription_callback, 10)
        self.subscription_dict['point'] = self.create_subscription(RobotPoint, 'robot_point', self.subscription_callback, 10)
        self.subscription_dict['orientation'] = self.create_subscription(RobotQuaternion, 'robot_orientation', self.subscription_callback, 10)
        self.subscription_dict['misc'] = self.create_subscription(RobotMisc, 'robot_misc', self.subscription_callback, 10)
        self.subscription_dict['neighbors'] = self.create_subscription(NeighborList, 'neighbors', self.subscription_callback, 10)
        self.get_logger().debug('Subscriptions initialized')

        # create activity_sub if DB defines states
        if self.connect_db():
            with self.conn.cursor() as cursor:
                cursor.execute("SELECT * FROM State")
                if cursor.rowcount:
                    for row in cursor.fetchall():
                        self.__state_map[row[1]] = row[0]
                    # self.__activity_sub = self.create_subscription(RobotActivity, 'robot_activity', self.subscription_callback)

    # properties
    @property
    def nid_map(self):
        return self.__nid_map
    @property
    def state_map(self):
        return self.__state_map

    def forward_batch_test(self):
        for nid in self.nodes.keys():
            output = nid
            for key in self.nodes[nid].keys():
                output += f'\n\t{key}: {self.nodes[nid][key]}'
            print(output, end="\n\n")
    
    # methods
    def register_new_nodes(self, cursor):
        unregistered = False
        # find nids without robot_id
        try:
            for node in self.nodes.keys():
                if node not in self.db_robot_ids.keys():
                    cursor.execute('INSERT INTO %s (%s) VALUES (%s)', (
                        DB_TABLE_NAMES['robot'],
                        DB_COLUMN_NAMES['nid'],
                        node,
                    ))
                    unregistered = True
            # if at least one is found
            if unregistered:
                self.conn.commit()
                # get all robot_ids
                cursor.execute('SELECT %s, %s FROM %s', (
                    DB_COLUMN_NAMES['robot_id'],
                    DB_COLUMN_NAMES['nid'],
                    DB_TABLE_NAMES['robot']
                ))
                # update nid-to-robot_id map
                if not cursor.rowcount:
                    self.get_logger().error('Failed to forward batch: No robots found')
                    return
                for row in cursor.fetchall(): 
                    self.nid_map[row[1]] = row[0]
        except Exception as e:
            self.get_logger().error('Failed to update registered Robots: %s' % (e,))

    def forward_batch(self):
        tries = 5
        while tries != 0:
            if self.connect_db():
                try:
                    t = TimeUtil.get_timestamp()
                    with self.conn.cursor() as cursor:
                        self.register_new_nodes(cursor)
                        for nid in self.nodes.keys():
                            if t - self.nodes[nid]['last'] < 30:
                                cursor.execute("""INSERT INTO %s (%s, %s, %s, %s, %s, %s, %s, %s)
                                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s)""", (
                                        # table name
                                        DB_TABLE_NAMES['status'],
                                        # column names
                                        DB_COLUMN_NAMES['robot_id'],
                                        DB_COLUMN_NAMES['battery'],
                                        DB_COLUMN_NAMES['cpu'],
                                        DB_COLUMN_NAMES['point'],
                                        DB_COLUMN_NAMES['orientation'],
                                        DB_COLUMN_NAMES['ipv4'],
                                        DB_COLUMN_NAMES['ipv6'],
                                        DB_COLUMN_NAMES['mac'],
                                        # values
                                        self.nid_map[nid],
                                        self.nodes[nid]['battery'],
                                        self.nodes[nid]['cpu'],
                                        self.nodes[nid]['point'],
                                        self.nodes[nid]['orientation'],
                                        self.nodes[nid]['ipv4'],
                                        self.nodes[nid]['ipv6'],
                                        self.nodes[nid]['mac'],
                                    )
                                )
                                for neighbor in self.nodes[nid]['neighbors'].keys():
                                    cursor.execute("""INSERT INTO %s (%s, %s, %s)
                                        VALUES (%s, %s, %s)""", (
                                            # table name
                                            DB_TABLE_NAMES['neighbor'],
                                            # column names
                                            DB_COLUMN_NAMES['robot_id'],
                                            DB_COLUMN_NAMES['neighbor'],
                                            DB_COLUMN_NAMES['strength'],
                                            # values
                                            self.__nid_map[nid],
                                            self.__nid_map[neighbor],
                                            self.nodes[nid]['neighbors'][neighbor],
                                        ))

                        self.conn.commit()
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
        print('Batch forwarded: %s' % (TimeUtil.get_datetime_f(),))
    
    def check_nid(self, nid) -> bool: # override
        if nid and not nid in self.nodes.keys():
            self.nodes[nid] = {
                'battery': None,
                'cpu': None,
                'activity': None,
                'point': None,
                'orientation': None,
                'ipv4': None,
                'ipv6': None,
                'mac': None,
                'neighbors': {}
            }
        return True

    # subscription callbacks
    def subscription_callback(self, msg):
        if self.check_nid(msg.header.nid):
            self.nodes[msg.header.nid]['last'] = msg.header.time.sec
            match type(msg).__name__:
                case 'RobotBattery':
                    self.nodes[msg.header.nid]['battery'] = msg.data
                case 'RobotCpu':
                    self.nodes[msg.header.nid]['cpu'] = msg.data
                case 'RobotActivity':
                    # check if activity_sub exists
                    if 'activity' in self.subscription_dict.keys():
                        # check if activity has changed
                        if self.nodes[msg.header.nid]['activity'] != msg.activity:
                            # if changed, update activity locally and in DB
                            self.nodes[msg.header.nid]['activity'] = msg.activity
                            if self.connect_db():
                                try:
                                    with self.conn.cursor() as cursor:
                                        cursor.execute("""UPDATE %s
                                            SET %s = %s
                                            WHERE %s = %s
                                            """, (
                                                    DB_TABLE_NAMES['robot'],
                                                    DB_COLUMN_NAMES['activity'],
                                                    self.__state_map[msg.activity],
                                                    DB_COLUMN_NAMES['robot_id'],
                                                    self.__nid_map[msg.header.nid],
                                                )
                                            )
                                except Error as e:
                                    self.get_logger().error('Failed to update registered Robots: %s' % (e,))
                                    try:
                                        self.conn.rollback()
                                    except:
                                        self.get_logger().error('Error: Rollback failed')
                case 'RobotPoint':
                    self.nodes[msg.header.nid]['point'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}
                case 'RobotQuaternion':
                    self.nodes[msg.header.nid]['orientation'] = {'x': msg.x, 'y': msg.y, 'z': msg.z, 'w': msg.w}
                case 'RobotMisc':
                    if msg.ipv4:
                        self.nodes[msg.header.nid]['ipv4'] = msg.ipv4
                    if msg.ipv6:
                        self.nodes[msg.header.nid]['ipv6'] = msg.ipv6
                    if msg.mac:
                        self.nodes[msg.header.nid]['mac'] = msg.mac
                case 'NeighborList':
                    for i in range(0, len(msg.neighbors)):    
                        # from nodes > get publisher > get neighbors > access i-th neighbor 
                        self.nodes[msg.header.nid]['neighbors'][msg.neighbors[i]] = msg.indicators[i]
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
