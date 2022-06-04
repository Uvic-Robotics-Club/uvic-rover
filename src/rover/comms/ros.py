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
import cv2 as cv
from cv_bridge import CvBridge
import rospy
from rover.comms.commands import CommandType
from rover.comms.connection_client import ConnectionClient
from rover.comms.exceptions import NoConnectionException
from rover.msg import Speed, Coordinates, Arm
from sensor_msgs.msg import Image

#telemetry_state = TelemetryState()
bridge = CvBridge()

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
    def publish_arm_msg(command_params):
        assert type(command_params['x_axis_velocity']) == float
        assert type(command_params['y_axis_velocity']) == float
        assert type(command_params['z_axis_velocity']) == float
        assert type(command_params['gripper_velocity']) == float
        assert type(command_params['gripper_rotation_velocity']) == float

        msg = Arm()
        msg.x_axis_velocity = command_params['x_axis_velocity']
        msg.y_axis_velocity = command_params['y_axis_velocity']
        msg.z_axis_velocity = command_params['z_axis_velocity']
        msg.gripper_velocity = command_params['gripper_velocity']
        msg.gripper_rotation_velocity = command_params['gripper_rotation_velocity']

        publishers['arm'].publish(msg)

    @staticmethod
    def subscribe_gps_coordinates(data):
        assert type(data) == Coordinates

        telemetry_data = {
            'GPS': {
                'latitude': data.latitude,
                'longitude': data.longitude,
                'altitude_msl': data.altitude_msl,
                'mode': data.mode,
                'message': data.message
            }
        }

        rospy.loginfo('Received GPS data')

        # telemetry_state.set_telemetry_value('gps_coordinates', telemetry_data)

        try:
            ConnectionClient.send_telemetry(telemetry_data)
        except NoConnectionException:
            rospy.loginfo('No connection...')
            # If there is no connection, then ignore.
            pass

    @staticmethod
    def subscribe_camera_image(data):
        assert type(data) == Image

        data = cv.imencode('.jpg', bridge.imgmsg_to_cv2(data))[1].tostring()

        try:
            ConnectionClient.send_camera_frame(data)
        except NoConnectionException:
            # If there is not connection, then ignore.
            pass

PUBLISHERS_PARAMS = {
    'speed': {'data_class': Speed, 'queue_size': 10},
    'arm': {'data_class': Arm, 'queue_size': 10}
}

'''
Comms node subscribes to topics, primarily for the purpose of publishing telemetry 
to the base station.
'''
SUBSCRIBERS_PARAMS = {
    'gps_coordinates': {'data_class': Coordinates, 'callback_func': ROS.subscribe_gps_coordinates},
    'camera_image': {'data_class': Image, 'callback_func': ROS.subscribe_camera_image}
}

publishers = {}
for pub_name in PUBLISHERS_PARAMS:
    publishers[pub_name] = rospy.Publisher(name=pub_name, data_class=PUBLISHERS_PARAMS[pub_name]['data_class'], queue_size=PUBLISHERS_PARAMS[pub_name]['queue_size'])

subscribers = {}
for sub_name in SUBSCRIBERS_PARAMS:
    subscribers[sub_name] = rospy.Subscriber(name=sub_name, data_class=SUBSCRIBERS_PARAMS[sub_name]['data_class'], callback=SUBSCRIBERS_PARAMS[sub_name]['callback_func'])
