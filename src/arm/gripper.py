#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class GripperControl:
    def __init__(self):
        self.pub_button0 = rospy.Publisher('gripper_control_button0', Bool, queue_size=10)
        self.pub_button1 = rospy.Publisher('gripper_control_button1', Bool, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.loginfo("Gripper control node initialized")

    def joy_callback(self, data):
        
        button0_state = Bool(data.buttons[0])
        button1_state = Bool(data.buttons[1])
        
        rospy.loginfo("Sending button0 state: %s", button0_state.data)
        rospy.loginfo("Sending button1 state: %s", button1_state.data)
        
        self.pub_button0.publish(button0_state)
        self.pub_button1.publish(button1_state)

if __name__ == '__main__':
    rospy.init_node('gripper_control')
    gripper_control = GripperControl()
    rospy.spin()
