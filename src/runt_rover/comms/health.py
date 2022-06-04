'''
This file contians code which is responsible for ensuring that the
connection to the rover is still alive.
'''
import requests
import rospy
from runt_rover.comms.connection_client import ConnectionClient
from runt_rover.comms.exceptions import NoConnectionException
from runt_rover.comms.state import NodeState
import threading
from time import sleep, time

node_state = NodeState()

class HealthCheck():

    @staticmethod
    def monitor_connection():
        '''
        Monitors current connection, if any, as part of health
        checks.
        '''
        if not node_state.get_attribute('connection_established'):
            return
        thread_id = threading.get_ident()
        connection_id = node_state.get_attribute('connection_id')
        rospy.loginfo('New health check thread (thread ID: {}) for connection ID: {}'.format(thread_id, connection_id))

        last_response_timestamp = float('inf')
        while (time() - last_response_timestamp) < rospy.get_param("/comms_healthcheck_timeout_rover_connection_sec"):

            # Wait for ping interval, then ping rover.
            sleep(rospy.get_param('/comms_healthcheck_interval_sec'))

            # If connection ID is different, then return.
            if connection_id != node_state.get_attribute('connection_id'):
                rospy.loginfo('Health check: Connection ID has changed, ending health check thread (thread ID: {})'.format(thread_id))
                return

            # If connection is now closed, the return.
            if not node_state.get_attribute('connection_established'):
                rospy.loginfo('Health check: Connection closed by another thread (thread ID: {})'.format(thread_id))
                return

            try:
                ConnectionClient.ping()
                rospy.loginfo('Health check (thread ID: {}): Ping success!'.format(thread_id))
                last_response_timestamp = time()
            except requests.exceptions.ConnectionError as err:
                rospy.loginfo('Health check (thread ID: {}): Ping failed, unable to reach base station'.format(thread_id))
            except requests.exceptions.Timeout as ex:
                rospy.loginfo('Health check (thread ID: {}): Ping request timed out'.format(thread_id))
            except AssertionError as err:
                rospy.loginfo('Health check (thread ID: {}): Response code from ping is not 200 OK'.format(thread_id)) 

        # Forcibly disconnect from the rover.
        try:
            # Send request to rover, with short timeout. Ideally, would send
            # async request, but requests module does not have support currently.
            rospy.loginfo('Timeout, disconnecting from base station (connection ID: {}, thread ID: {})'.format(connection_id, thread_id))
            ConnectionClient.disconnect()
        except NoConnectionException:
            pass
        except requests.exceptions.ConnectionError:
            pass
        except requests.exceptions.Timeout:
            pass
        except AssertionError:
            pass