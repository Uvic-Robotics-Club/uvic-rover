import requests
from runt_rover.comms.state import NodeState

REQUEST_TIMEOUT_SEC = 5.0

node_state = NodeState()

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
            raise Exception('No connection established.')

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
            raise Exception('No connection established.')

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
    def disconnect():
        '''
        Disconnects from the base station server. This indicates to the rover that the
        base station is not actively processing requests from the rover.
        '''

        if not node_state.get_attribute('connection_established'):
            raise Exception('No connection established.')

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