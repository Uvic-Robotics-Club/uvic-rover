import requests
from runt_rover.comms.state import State

class ConnectionClient():
    def __init__(self):
        self.state = State()

    def connect_to_http_host(self, host_address):
        '''
        Attempts to establish an HTTP connection to the base station server. The
        'connection' is an abstraction on top of the HTTP requests, which allows the
        base station to be aware of the history of HTTP requests between the rover
        and base station.
        '''
        assert(type(host_address)) == str

        try:
            request_url = '{}/connect'.format(host_address)
            response = requests.get(request_url)
            assert response.status_code == 200
        except AssertionError as err:
            pass

        json_data = response.json()
        self.state.set_attribute('connection_established', True)
        self.state.set_attribute('connection_remote_addr', host_address)
        self.state.set_attribute('connection_id', json_data['connection_id'])

    def send_telemetry(self, data):
        '''
        Sends telemetry data to the base station, if a connection exists
        '''
        assert(type(data)) == dict

        if not self.state.get_attribute('connection_established'):
            return False # TODO raise exception?

        try:
            request_url = '{}/send_telemetry'.format(self.state.get_attribute('connection_remote_addr'))
            response = requests.post(request_url, json=data)
            assert response.status_code == 200
        except AssertionError as err:
            pass