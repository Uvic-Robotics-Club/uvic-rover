from ast import Assert
from flask import Flask, request
import requests
import rospy
from runt_rover.comms.commands import CommandType, CommandParser
from runt_rover.comms.state import NodeState, TelemetryState
from runt_rover.comms.ros import ROS
import threading

node_state = NodeState()
telemetry_state = TelemetryState()
base_station_port = rospy.get_param("/base_station_port")

# Create and configure server
app = Flask(__name__, instance_relative_config=True)
app.config.from_mapping(
    SECRET_KEY='dev'
)

@app.route('/')
def root():
    return {'status': 'success', 'message': 'Rover server is alive!'}

@app.route('/connect', methods=['GET'])
def connect():
    '''
    This method is called by the base station to begin a connection.
    '''
    response = {'status': None}
    args = dict(request.args)
    remote_addr = request.remote_addr

    try:
        assert 'conn_id' in args, 'conn_id parameter not provided.'
    except AssertionError as err:
        response['status'] = 'Malformed request'
        response['message'] = str(err)
        return response

    # Fail if connection is already established
    # TODO: Determine if this is appropriate
    #if state.get_attribute('connection_established'):
    #    response['status'] = 'failure'
    #    response['message'] = 'Connection already exists, cannot start new connection.'
    #    return response

    new_connection_id = int(args['conn_id'])
    node_state.set_attribute('connection_id', new_connection_id)
    node_state.set_attribute('connection_remote_addr', remote_addr)
    node_state.set_attribute('connection_established', True)
    node_state.set_attribute('connection_port', base_station_port)

    response['status'] = 'success'
    response['message'] = 'Established connection with ID {}'.format(new_connection_id)
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
    except AssertionError as err:
        response['status'] = 'failure'
        response['message'] = str(err)

    # Fail if connection is not established.
    if not node_state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'No connection established, unable to process command.'
        return response

    # Ensure request comes from correct remote addr.
    if node_state.get_attribute('connection_remote_addr') != remote_addr:
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
    try:
        command_params = CommandParser.parse_command(command_type, args)
    except KeyError as err:
        response['status'] = 'failure'
        response['message'] = 'Failed to find key: {} in {} command.'.format(str(err), args['type'])
        return response

    ROS.publish_command(command_type, command_params)

    response['status'] = 'success'
    response['message'] = 'Published command to all nodes.'
    return response

@app.route('/get_rover_telemetry', methods=['GET'])
def get_rover_telemetry():
    '''
    This method is called by the base station to fetch the latest telemetry
    data stored for the rover.
    '''
    response = {'status': None}
    remote_addr = request.remote_addr

    if not node_state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'No established connection exists.'
        return response

    if node_state.get_attribute('connection_remote_addr') != remote_addr:
        response['status'] = 'failure'
        response['message'] = 'Cannot terminate connection from another host.'

    response['data'] = telemetry_state.get_all_attributes()
    return response

@app.route('/disconnect', methods=['GET'])
def disconnect():
    '''
    This method is called by the base station to terminate the connection
    to the base station, if any. Ensure only the address associated with the
    connection is able to terminate the connection.
    '''
    response = {'status': None}
    remote_addr = request.remote_addr

    if not node_state.get_attribute('connection_established'):
        response['status'] = 'failure'
        response['message'] = 'No established connection exists.'
        return response

    if node_state.get_attribute('connection_remote_addr') != remote_addr:
        response['status'] = 'failure'
        response['message'] = 'Cannot terminate connection from another host.'

    node_state.set_attribute('connection_established', False)
    node_state.set_attribute('connection_remote_addr', None)
    node_state.set_attribute('connection_id', None)

    response['status'] = 'success'
    response['message'] = 'Terminated connection to rover.'
    return response


if __name__ == '__main__':
    app.run()