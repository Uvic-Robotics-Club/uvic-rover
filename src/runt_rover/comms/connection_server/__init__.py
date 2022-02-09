from flask import Flask, request
import os
from runt_rover.comms.connection_client import ConnectionClient
from runt_rover.comms.state import State

connection_client = ConnectionClient()
state = State()

# Create and configure server
app = Flask(__name__, instance_relative_config=True)
app.config.from_mapping(
    SECRET_KEY='dev'
)

@app.route('/')
def root():
    return {'status': 'success', 'message': 'Rover server is alive!'}

@app.route('/', methods=['GET'])
def request_connection():
    '''
    This method is called by the base station to request the rover to 
    connect to the base station.
    '''
    response = {'status': None}
    remote_addr = request.remote_addr

    if state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'Connection already exists, cannot start new connection.'
        return response

    connection_client.connect_to_http_host(remote_addr)

if __name__ == '__main__':
    app.run()