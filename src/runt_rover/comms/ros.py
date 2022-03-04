'''
This file contains ROS constructs which allow the comms node to communicate with other nodes.
Communication includes relaying commands from the base station and fetching telemetry and video
from the rover.

The comms node uses both topics (publishers & subscribers), as well as services (services and clients), 
depending on the data required:
  1) When comms node receives command from base station, publishes it for any interested node that needs 
     to process the command.
  2) When comms node would like to send telemetry to the base station, it uses clients to fetch data from
     other nodes about the state of the rover.
'''
import rospy
from runt_rover.comms.commands import CommandType
from runt_rover.msg import Speed, Coordinates
from runt_rover.comms.state import TelemetryState

telemetry_state = TelemetryState()

PUBLISHERS_PARAMS = {
    'speed': {'data_class': Speed, 'queue_size': 10}
}

SUBSCRIBERS_PARAMS = {
    'gps_coordinates': {'data_class': Coordinates, 'callback_func': (lambda x: x)}
}

publishers = {}
for pub_name in PUBLISHERS_PARAMS:
    publishers[pub_name] = rospy.Publisher(name=pub_name, data_class=PUBLISHERS_PARAMS[pub_name]['data_class'], queue_size=PUBLISHERS_PARAMS[pub_name]['queue_size'])

subscribers = {}
for sub_name in SUBSCRIBERS_PARAMS:
    subscribers[sub_name] = rospy.Subscriber(name=sub_name, data_class=SUBSCRIBERS_PARAMS[sub_name]['data_class'], callback=SUBSCRIBERS_PARAMS[sub_name]['callback_func'])

class ROS():

    @staticmethod
    def publish_command(command_type, command_params):
        assert type(command_type) == CommandType
        assert type(command_params) == dict

        if command_type == CommandType.DRIVE_TRAIN:
            ROS.publish_drive_train_msg(command_params)

    @staticmethod
    def publish_drive_train_msg(command_params):
        assert type(command_params['left_speed']) == int
        assert type(command_params['right_speed']) == int
        assert type(command_params['left_direction']) == int
        assert type(command_params['right_direction']) == int

        msg = Speed()
        msg.leftspeed = command_params['left_speed']
        msg.rightspeed = command_params['right_speed']
        msg.leftdirection = command_params['left_direction']
        msg.rightdirection = command_params['right_direction']

        publishers['speed'].publish(msg)

    @staticmethod
    def subscribe_gps_coordinates(data):
        assert type(data) == Coordinates

        coords = {
            'latitude': data.latitude,
            'longitude': data.longitude,
            'altitude_msl': data.altitude_msl,
            'mode': data.mode,
            'message': data.message
        }

        telemetry_state.set_telemetry_value('gps_coordinates', coords)
