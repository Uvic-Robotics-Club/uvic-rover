from ast import Assert
from flask import Flask, request
import requests
import rospy
from runt_rover.comms.commands import CommandType, CommandParser
from runt_rover.comms.connection_client import ConnectionClient
from runt_rover.comms.state import State
from runt_rover.comms.ros import ROS

state = State()
base_station_port = rospy.get_param("/base_station_port")

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
        ConnectionClient.connect_to_http_host(remote_addr, base_station_port)
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

@app.route('/send_command', methods=['POST'])
def send_command():
    '''
    This method is called by the base stattion to send a command to
    the rover.
    '''
    response = {'status': None}
    remote_addr = request.remote_addr
    args = request.json

    try:
        assert 'type' in args, 'type argument not provided.'
        assert 'params' in args, 'params argument not provided.'
    except AssertionError as err:
        response['status'] = 'failure'
        response['message'] = str(err)

    # Fail if connection is not established.
    if not state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'No connection established, unable to process command.'
        return response

    # Ensure request comes from correct remote addr.
    if state.get_attribute('connection_remote_addr') != remote_addr:
        response['status'] = 'failure'
        response['message'] = 'Remote address does not match current connection base station host address.'
        return response

    # Validate that command is recognized.
    try:
        command_type = CommandType(args['type'])
    except ValueError:
        response['status'] = 'failure'
        response['message'] = 'Command Type "{}" not recognized.'.format(args['type'])
        return response

    # Parse command, and publish to ROS
    command_params = CommandParser.parse_command(command_type, args['params'])
    ROS.publish_command(command_type, command_params)


if __name__ == '__main__':
    app.run()