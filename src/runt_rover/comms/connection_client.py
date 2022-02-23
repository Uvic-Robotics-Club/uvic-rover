import requests
from runt_rover.comms.state import State

state = State()

class ConnectionClient():

    @staticmethod
    def connect(host_address, port):
        '''
        Attempts to establish an HTTP connection to the base station server. The
        'connection' is an abstraction on top of the HTTP requests, which allows the
        base station to be aware of the history of HTTP requests between the rover
        and base station.
        '''
        assert(type(host_address)) == str
        assert(type(port)) == int

        try:
            request_url = 'http://{}:{}/rover/connect'.format(host_address, port)
            response = requests.get(request_url, timeout=5.0)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

        json_data = response.json()
        state.set_attribute('connection_established', True)
        state.set_attribute('connection_remote_addr', host_address)
        state.set_attribute('connection_id', json_data['connection_id'])
        return True

    @staticmethod
    def send_telemetry(data, port):
        '''
        Sends telemetry data to the base station, if a connection exists
        '''
        assert(type(data)) == dict
        assert(type(port)) == int

        if not state.get_attribute('connection_established'):
            return False # TODO raise exception?

        try:
            request_url = 'http://{}:{}/rover/send_telemetry'.format(state.get_attribute('connection_remote_addr'), port)
            response = requests.post(request_url, json=data, timeout=5.0)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

    @staticmethod
    def disconnect(port):
        '''
        Disconnects from the base station server. This indicates to the rover that the
        base station is not actively processing requests from the rover.
        '''
        assert type(port) == int

        if not state.get_attribute('connection_established'):
            return False # TODO raise exception?

        try:
            request_url = 'http://{}:{}/rover/disconnect'.format(state.get_attribute('connection_remote_addr'), port)
            response = requests.get(request_url, timeout=5.0)
            assert response.status_code == 200
        except requests.exceptions.Timeout as ex:
            raise ex
        except requests.exceptions.ConnectionError as err1:
            raise err1
        except AssertionError as err2:
            raise err2

        state.set_attribute('connection_established', False)
        state.set_attribute('connection_remote_addr', None)
        state.set_attribute('connection_id', None)