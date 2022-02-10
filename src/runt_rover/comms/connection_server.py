from flask import Flask, request
import requests
from runt_rover.comms.connection_client import ConnectionClient
from runt_rover.comms.state import State

state = State()

# Create and configure server
app = Flask(__name__, instance_relative_config=True)
app.config.from_mapping(
    SECRET_KEY='dev'
)

@app.route('/')
def root():
    return {'status': 'success', 'message': 'Rover server is alive!'}

@app.route('/request_connection', methods=['GET'])
def request_connection():
    '''
    This method is called by the base station to request the rover to 
    connect to the base station.
    '''
    response = {'status': None}
    remote_addr = request.remote_addr

    # Fail if connection is already established
    if state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'Connection already exists, cannot start new connection.'
        return response

    try:
        ConnectionClient.connect_to_http_host(remote_addr)
    except requests.exceptions.Timeout as ex:
        response['status'] = 'failure'
        response['message'] = 'Timeout: connection to base station failed.'
        return response
    except requests.exceptions.ConnectionError as err1:
        response['status'] = 'failure'
        response['message'] = 'Connection error: Most likely connection refused due to port not being open.'
        return response
    except AssertionError as err2:
        response['status'] = 'failure'
        response['message'] = 'Response status code to connect to base station is not 200.'
        return response

    response['status'] = 'success'
    response['message'] = 'Sent connection request to base station.'
    return response

if __name__ == '__main__':
    app.run()