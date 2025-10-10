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

from json import loads, dumps
from time import sleep
from sw_robot.util.time_util import TimeUtil
from sw_robot.util.sw_util import BaseStatusSub
from psycopg2 import Error
from os import getenv

FORWARD_TO_DB = getenv('FORWARD_TO_DB') # forward to DB unless not set to anything
env = getenv('TRIES_TO_GET_STATES')
TRIES_TO_GET_STATES = int(env) if env and env.isnumeric else 1

class RobotStatusSub(BaseStatusSub):
    def __init__(self):
        super().__init__()
        self.__nid_map = dict() # <nid>:<robot_id>
        self.__state_map = dict() # <activity>:<state_id>
        
        # subscriptions
        self.createSubscription('battery', RobotBattery, 'robot_battery', self.subscription_callback)
        self.createSubscription('cpu', RobotCpu, 'robot_cpu', self.subscription_callback)
        self.createSubscription('activity', RobotActivity, 'robot_activity', self.subscription_callback)
        self.createSubscription('point', RobotPoint, 'robot_point', self.subscription_callback)
        self.createSubscription('orientation', RobotQuaternion, 'robot_orientation', self.subscription_callback)
        self.createSubscription('misc', RobotMisc, 'robot_misc', self.subscription_callback)
        self.createSubscription('neighbors', NeighborList, 'neighbors', self.subscription_callback)
        self.get_logger().debug('Subscriptions initialized')

        # get states with IDs from DB
        for _ in range(TRIES_TO_GET_STATES):
            if self.connect_db():
                try:
                    with self.conn.cursor() as cursor:
                        cursor.execute('SELECT * FROM state')
                        if cursor.rowcount:
                            for row in cursor.fetchall():
                                # row[0] = id, row[1] = description
                                self.__state_map[row[1]] = row[0]
                    break
                except Exception as e:
                    self.get_logger().error('Failed to get states from DB: %s', (e,))
                    self.get_logger().info('Trying again in 10s')
                    sleep(10)
        if not len(self.__state_map):
            self.get_logger().warning('Failed to get states from DB multiple times. Continuing without any state definitions.')
    

    # properties
    @property
    def nid_map(self):
        return self.__nid_map
    
    @property
    def state_map(self):
        return self.__state_map


    # methods
    def forward_batch_test(self):
        try:
            with open('forward_batch_test_output', 'w') as f:
                for nid in self.nodes.keys():
                    output = nid
                    for key in self.nodes[nid].keys():
                        output += f'\n\t{key}: {self.nodes[nid][key]}'
                    print(output, end="\n\n")
                    f.write(output + '\n\n')
        except:
            self.get_logger().error('Failed during execution of forward_batch_test')
    
    def register_new_nodes(self, cursor):
        unregistered = False
        # find nids without robot_id
        try:
            for node in self.nodes.keys():
                if node not in self.nid_map.keys():
                    cursor.execute('INSERT INTO robot (nid, ipv4, ipv6, mac) VALUES (%s, %s, %s, %s)', (
                        node,
                        self.nodes[node]['ipv4'],
                        self.nodes[node]['ipv6'],
                        self.nodes[node]['mac'],
                    ))
                    unregistered = True
            # if at least one is found
            if unregistered:
                self.conn.commit()
                # get all robot_ids
                cursor.execute('SELECT robot_id, nid FROM robot')
                # update nid-to-robot_id map (nid_map)
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
                                cursor.execute('''INSERT INTO status (robot_id, battery, cpu_1, point, orientation, last_heard)
                                    VALUES (%s, %s, %s, %s, %s, %s)''', (
                                        self.nid_map[nid],
                                        self.nodes[nid]['battery'],
                                        self.nodes[nid]['cpu'],
                                        dumps(self.nodes[nid]['point']),
                                        dumps(self.nodes[nid]['orientation']),
                                        self.nodes[nid]['last'],
                                    )
                                )
                                for neighbor in self.nodes[nid]['neighbors'].keys():
                                    if neighbor in self.__nid_map.keys():
                                        cursor.execute('''INSERT INTO neighbor (robot_id, neighbor, strength)
                                            VALUES (%s, %s, %s)''', (
                                                self.__nid_map[nid],
                                                self.__nid_map[neighbor],
                                                self.nodes[nid]['neighbors'][neighbor],
                                            ))

                        self.conn.commit()
                        return
                except Error as e: # psycopg2.Error
                    self.get_logger().error('SQL error: %s' % (e,))
                    try:
                        self.conn.rollback()
                    except:
                        self.get_logger().error('Error: Rollback failed')
                except Exception as e:
                    self.get_logger().error('Error: %s' % (e,))
                    return
            else:
                tries -= 1
                sleep(0.2)


    # timer callbacks
    def batch_timer_callback(self):
        if not FORWARD_TO_DB:
            self.forward_batch_test() # print
        else:
            self.forward_batch() # forward to DB
        # print('Batch forwarded: %s' % (TimeUtil.get_datetime_f(),))
        self.get_logger().info('Batch forwarded: %s' % (TimeUtil.get_datetime_f(),))
    
    def check_nid(self, nid) -> bool: # override; allows all
        if nid and nid not in self.nodes.keys():
            self.nodes[nid] = {
                'battery': None,
                'cpu': None,
                'activity': None,
                'point': None,
                'orientation': None,
                'ipv4': None,
                'ipv6': None,
                'mac': None,
                'neighbors': {},
                'last': None
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
                    # check if activity_sub exists and if activity is legal
                    if self.getSubscription('activity') and msg.activity in self.state_map.keys():
                        # check if activity has changed
                        if self.nodes[msg.header.nid]['activity'] != msg.activity:
                            # if changed, update activity locally and in DB
                            self.nodes[msg.header.nid]['activity'] = msg.activity
                            if self.connect_db():
                                try:
                                    with self.conn.cursor() as cursor:
                                        cursor.execute('''UPDATE robot
                                            SET activity = %s
                                            WHERE robot_id = %s
                                            ''', (
                                                    self.__state_map[msg.activity],
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
                    point = {'x': msg.x, 'y': msg.y, 'z': msg.z}
                    if self.validate_point(point):
                        self.nodes[msg.header.nid]['point'] = point
                case 'RobotQuaternion':
                    quaternion = {'x': msg.x, 'y': msg.y, 'z': msg.z, 'w': msg.w}
                    if self.validate_quaternion(quaternion):
                        self.nodes[msg.header.nid]['orientation'] = quaternion
                case 'RobotMisc':
                    if self.validate_ipv4(msg.ipv4):
                        self.nodes[msg.header.nid]['ipv4'] = msg.ipv4
                    if self.validate_ipv6(msg.ipv6):
                        self.nodes[msg.header.nid]['ipv6'] = msg.ipv6
                    if self.validate_mac(msg.mac):
                        self.nodes[msg.header.nid]['mac'] = msg.mac
                case 'NeighborList':
                    for i in range(0, len(msg.neighbors)):
                        if i == len(msg.indicators):
                            break
                        # from nodes > get publisher > get neighbors > access i-th neighbor
                        if self.validate_neighbor(msg.neighbors[i]) and self.validate_indicator(msg.indicators[i]):
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
