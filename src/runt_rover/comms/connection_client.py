import math
import requests
import rospy
from runt_rover.comms.exceptions import NoConnectionException
from runt_rover.comms.state import NodeState
import socket
import struct

REQUEST_TIMEOUT_SEC = 5.0
MAX_DGRAM_SIZE_BYTES = 2**16 - 64

node_state = NodeState()
datagram_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
base_station_video_port = rospy.get_param("/base_station_video_port")

class ConnectionClient():

    @staticmethod
    def connect(host_address, port):
        '''
        Attempts to establish an HTTP connection to the base station server. The
        'connection' is an abstraction on top of the HTTP requests, which allows the
        base station to be aware of the history of HTTP requests between the rover
        and base station.
        '''
        assert type(host_address) == str
        assert type(port) == int

        try:
            request_url = 'http://{}:{}/api/rover/connect'.format(host_address, port)
            response = requests.get(request_url, timeout=REQUEST_TIMEOUT_SEC)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

        json_data = response.json()
        node_state.set_attribute('connection_established', True)
        node_state.set_attribute('connection_remote_addr', host_address)
        node_state.set_attribute('connection_port', port)
        node_state.set_attribute('connection_id', json_data['connection_id'])

        return True

    @staticmethod
    def send_telemetry(data):
        '''
        Sends telemetry data to the base station, if a connection exists. Current implementation returns all current 
        telemetry data for each request. May have to send in smaller intervals if data becomes too large.
        '''
        assert type(data) == dict

        if not node_state.get_attribute('connection_established'):
            raise NoConnectionException

        try:
            request_url = 'http://{}:{}/api/rover/send_telemetry'.format(node_state.get_attribute('connection_remote_addr'), node_state.get_attribute('connection_port'))
            response = requests.post(request_url, json=data, timeout=REQUEST_TIMEOUT_SEC)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

    @staticmethod
    def ping():
        '''
        Pings the base station if a connection exists. If response has status code 200 (OK),
        successds call.
        '''

        if not node_state.get_attribute('connection_established'):
            raise NoConnectionException

        try:
            request_url = 'http://{}:{}/'.format(node_state.get_attribute('connection_remote_addr'), node_state.get_attribute('connection_port'))
            response = requests.get(request_url, timeout=REQUEST_TIMEOUT_SEC)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

    @staticmethod
    def send_camera_frame(data):
        '''
        Sends an image represented by a string, data, to the base station by UDP. UDP is used
        as image quality loss is accepted.
        '''
        #assert type(data) == str
        #print('Length to send: {}'.format(len(data)))

        if not node_state.get_attribute('connection_established'):
            raise NoConnectionException

        # Determine number of segments to send.
        size = len(data)
        num_of_segments = math.ceil(size / MAX_DGRAM_SIZE_BYTES)

        # Send each segment individually, delimited by index start and end.
        array_index_start = 0

        # curr_segment is a delimiter for the number of segments per image.
        for curr_segment in range(num_of_segments-1, -1, -1):
            array_index_end = min(size, array_index_start + MAX_DGRAM_SIZE_BYTES)
            
            #print('Segment sent, length: {}, index: {}'.format(array_index_end-array_index_start, curr_segment))
            datagram_socket.sendto(
                struct.pack("B", curr_segment) + data[array_index_start: array_index_end],
                (node_state.get_attribute('connection_remote_addr'), base_station_video_port)
            )
            array_index_start = array_index_end

    @staticmethod
    def disconnect():
        '''
        Disconnects from the base station server. This indicates to the rover that the
        base station is not actively processing requests from the rover.
        '''

        if not node_state.get_attribute('connection_established'):
            raise NoConnectionException

        try:
            request_url = 'http://{}:{}/api/rover/disconnect'.format(node_state.get_attribute('connection_remote_addr'), node_state.get_attribute('connection_port'))
            response = requests.get(request_url, timeout=REQUEST_TIMEOUT_SEC)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

        node_state.set_attribute('connection_established', False)
        node_state.set_attribute('connection_remote_addr', None)
        node_state.set_attribute('connection_port', None)
        node_state.set_attribute('connection_id', None)