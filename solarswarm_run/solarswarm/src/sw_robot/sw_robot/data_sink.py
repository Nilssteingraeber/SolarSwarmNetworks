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
from sw_robot.util.sw_util import BaseStatusSub, Util
from psycopg2 import Error
from os import getenv
import logging

FORWARD_TO_DB = getenv('FORWARD_TO_DB')
DB_CONNECT_RETRIES = int(getenv('DB_CONNECT_RETRIES', '30'))
DB_CONNECT_DELAY = float(getenv('DB_CONNECT_DELAY', '1.0'))
DATA_STALENESS_CUTOFF = float(getenv('DATA_STALENESS_CUTOFF', '30.0'))

ENV = getenv('TRIES_TO_GET_STATES')
TRIES_TO_GET_STATES = int(ENV) if ENV and ENV.isnumeric() else DB_CONNECT_RETRIES
logging.basicConfig(
    filename=getenv('LOG_OUT'),
    level=logging.DEBUG,
    format="[data_sink] %(levelname)s: %(message)s (%(asctime)s)",
) # formatting options: https://docs.python.org/3/library/logging.html#logrecord-attributes
# usage: logging.info(<message>), logging.warning(<message>), ...

class RobotStatusSub(BaseStatusSub):
    def __init__(self):
        super().__init__()
        self.__nid_map = dict() # <nid>:<robot_id>
        self.__state_map = dict() # <activity>:<state_id>
        

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
                        cursor.execute('SELECT * FROM "State"')
                        for row in cursor.fetchall():
                            self.__state_map[row[1]] = row[0]
                    logging.info(f"Loaded {len(self.__state_map)} states.")
                    break
                except Exception as e:
                    logging.error('Failed to load states: %s', e)
                    try:
                        self.conn.rollback()
                    except Exception as e:
                        logging.error('Failed to rollback: %s', e)
                    sleep(DB_CONNECT_DELAY)
            else:
                logging.warning('DB not reachable while loading states')
                sleep(DB_CONNECT_DELAY)

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
                    logging.debug(output)
                    f.write(output + '\n\n')
        except:
            logging.error('Failed during execution of forward_batch_test.')
    
    def register_new_nodes(self, cursor):
        unregistered = False
        # find nids without robot_id
        try:
            # load current state from db
            cursor.execute('SELECT robot_id, nid FROM "Robot"')
            if not cursor.rowcount:
                logging.warning('Failed to load robots from db: No robots found.')
            for row in cursor.fetchall(): 
                self.nid_map[row[1]] = row[0]

            # insert missing robots
            for node in self.nodes.keys():
                if node not in self.nid_map.keys():


                    logging.warning(self.nodes, self.nodes[node], self.__state_map, node)
                    # local_data = self.nodes[node]
                    # target_state = self.__state_map.get(local_data.get('activity'))

                    cursor.execute('INSERT INTO "Robot" (nid, ipv4, ipv6, mac, state_id) VALUES (%s, %s, %s, %s, %s)', (
                        node,
                        self.nodes[node]['ipv4'],
                        self.nodes[node]['ipv6'],
                        self.nodes[node]['mac'],
                        1
                    ))
                    unregistered = True

            # if at least one was added, load again to get robot_id determined by db
            if unregistered:
                self.conn.commit()
                # get all robot_ids
                cursor.execute('SELECT robot_id, nid FROM \"Robot\"')
                # update nid-to-robot_id map (nid_map)
                if not cursor.rowcount:
                    logging.error('Failed to load robots from db: No robots found.')
                    return
                for row in cursor.fetchall(): 
                    self.nid_map[row[1]] = row[0]
        except Exception as e:
            logging.error('Failed to register robots: %s' % (e,))
            try:
                self.conn.rollback()
            except:
                logging.error('SQL error: Rollback failed')
    
    def update_existing_nodes(self, cursor):
        try:
            cursor.execute('SELECT robot_id, nid, state_id, display_name, ipv4, ipv6, mac FROM "Robot"')
            if cursor.rowcount:
                for row in cursor.fetchall():

                    nid = row[1]

                    if nid not in self.nid_map or nid not in self.nodes:
                        continue

                    local_data = self.nodes[nid]
                    
                    logging.error("##### Hi5! %s | %s - %s", local_data, row, local_data['activity'])

                    cursor.execute(
                        '''UPDATE "Robot"
                            SET ipv4=%s, ipv6=%s, mac=%s, state_id=%s
                            WHERE robot_id=%s''',
                        (
                            local_data['ipv4'],
                            local_data['ipv6'],
                            local_data['mac'],
                            local_data['activity'],
                            self.nid_map[nid]
                        )
                    )
                    logging.debug("Updated existing node with nid %s", (nid,))
                self.conn.commit()
            else:
                logging.warning('Table "Robot" contained no entries during update check.')

        except Error as e:
            logging.error('SQL error during update_existing_nodes: %s' % (e,))
            try:
                self.conn.rollback()
            except Exception as rb_e:
                logging.error('SQL rollback failed: %s' % (rb_e,))
        except Exception as e:
            logging.error('Unexpected error in update_existing_nodes: %s' % (repr(e),))

    def forward_batch(self):
        tries = DB_CONNECT_RETRIES

        while tries > 0:
            if not self.connect_db():
                tries -= 1
                logging.warning('DB connection failed. Retrying in %ss', DB_CONNECT_DELAY)
                sleep(DB_CONNECT_DELAY)
                continue

            try:
                t = TimeUtil.get_timestamp()
                with self.conn.cursor() as cursor:
                    self.register_new_nodes(cursor) # insert into table robot if necessary

                    for nid, data in self.nodes.items():
                        if nid not in self.__nid_map:
                            continue

                        # Check if data is fresh (within 30 seconds)
                        if data['last'] and (t - data['last'] < DATA_STALENESS_CUTOFF):
                            cursor.execute(
                                '''INSERT INTO "Status"
                                   (robot_id, battery, cpu_1, point, orientation, last_heard)
                                   VALUES (%s, %s, %s, %s, %s, %s)''',
                                (
                                    self.__nid_map[nid],
                                    data['battery'],
                                    data['cpu'],
                                    dumps(data['point']),
                                    dumps(data['orientation']),
                                    data['last'],
                                )
                            )

                            for neigh, strength in data['neighbors'].items():
                                if neigh in self.__nid_map:
                                    cursor.execute(
                                        '''INSERT INTO "Neighbor"
                                           (robot_id, neighbor, strength)
                                           VALUES (%s, %s, %s)''',
                                        (
                                            self.__nid_map[nid],
                                            self.__nid_map[neigh],
                                            strength,
                                        )
                                    )
                        else:
                            logging.info('Skipping node %s during batch forward: Not heard from recently.', nid)

                    self.conn.commit()
                    self.update_existing_nodes(cursor)
                return

            except Error as e: # specific psycopg2.Error handling
                logging.error('SQL error during batch forward: %s' % (e,))
                try:
                    self.conn.rollback()
                except Exception as rollback_err:
                    logging.error('SQL error: Rollback failed: %s' % (rollback_err,))
                return
            except Exception as e:
                logging.error('General error during batch forward: %s' % (e,))
                return


    def batch_timer_callback(self):
        # Instead of just checking and logging error, TRIGGER the reconnection logic
        if not self.connect_db():
            logging.error('Batch skipped: Database is down and reconnect failed.')
            return

        # If we reach here, connect_db() guaranteed us a live connection.
        if not FORWARD_TO_DB:
            self.forward_batch_test()
        else:
            self.forward_batch()
        # print('Batch forwarded: %s' % (TimeUtil.get_datetime_f(),))
        logging.info('Batch forwarded: %s' % (TimeUtil.get_datetime_f(),))

    
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
            logging.error(msg)
            logging.error(type(msg).__name__)
            self.nodes[msg.header.nid]['last'] = msg.header.time.sec
            match type(msg).__name__:
                case 'RobotBattery':
                    self.nodes[msg.header.nid]['battery'] = msg.data
                case 'RobotCpu':
                    self.nodes[msg.header.nid]['cpu'] = msg.data
                case 'RobotActivity':
                    # check if activity_sub exists and if activity is legal
                    logging.error("Activity subscription: %s, activity: %s, state_map keys: %s, nid: %s, nid_map keys: %s, state_map: %s",
                        self.getSubscription('activity'),
                        msg.activity,
                        list(self.state_map.keys()),
                        msg.header.nid,
                        list(self.nid_map.keys()),
                        self.__state_map[msg.activity])

                    if self.getSubscription('activity') and msg.activity in self.state_map.keys() and msg.header.nid in self.nid_map:
                        self.nodes[msg.header.nid]['activity'] = self.state_map[msg.activity]

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
                    # logging.debug('Received neighbors: %s', (str(msg.neighbors),))
                    # logging.debug('Received indicators: %s', (str(msg.indicators),))
                    if len(msg.neighbors) != len(msg.indicators):
                        logging.warning('Inequal length of neighors and indicators from node with nid %s', (msg.header.nid,))
                    for i in range(0, len(msg.neighbors)):
                        if i == len(msg.indicators):
                            break
                        # from nodes > get publisher > get neighbors > access i-th neighbor
                        if self.validate_neighbor(msg.neighbors[i]) and self.validate_indicator(msg.indicators[i]):
                            self.nodes[msg.header.nid]['neighbors'][Util.get_nid(msg.neighbors[i])] = msg.indicators[i]
                            # logging.debug(f'Added neighbor {Util.get_nid(msg.neighbors[i])} to node with nid {msg.header.nid}')
                case _:
                    logging.error('Caught message type without implemented match case: %s' % (type(msg).__name__))



def main():
    rclpy.init()
    robot_status_sub = RobotStatusSub()
    
    try:
        rclpy.spin(robot_status_sub)
    except KeyboardInterrupt:
        pass
    finally:
        if robot_status_sub.conn:
            try:
                robot_status_sub.conn.close()
                logging.info('DB connection closed gracefully.')
            except Exception as e:
                logging.error(f'Failed to close DB connection: {e}')
        
        robot_status_sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
