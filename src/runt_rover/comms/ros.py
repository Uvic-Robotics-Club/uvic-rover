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
from runt_rover.msg import DriveTrain

PUBLISHERS_PARAMS = {
    'drive_train': {'data_class': DriveTrain, 'queue_size': 10}
}

publishers = {}
for pub_name in PUBLISHERS_PARAMS:
    publishers[pub_name] = rospy.Publisher(name=pub_name, data_class=PUBLISHERS_PARAMS[pub_name]['data_class'], queue_size=PUBLISHERS_PARAMS[pub_name]['queue_size'])

class ROS():

    @staticmethod
    def publish_command(command_type, **command_params):
        assert type(command_type) == CommandType

        if command_type == CommandType.DRIVE_TRAIN:
            ROS.publish_drive_train_msg(**command_params)

    @staticmethod
    def publish_drive_train_msg(left_speed, right_speed, right_direction, left_direction):
        assert type(left_speed) == int
        assert type(right_speed) == int
        assert type(left_direction) == int
        assert type(right_direction) == int

        msg = DriveTrain()
        msg.leftspeed = left_speed
        msg.rightspeed = right_speed
        msg.rightdirection = right_direction
        msg.leftdirection = left_direction

        publishers['drive_train'].publish(msg)
