#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

def joy_callback(data):
    motor_command = Float32()
    motor_command.data = data.axes[4]  # Assuming the joystick's forward/backward axis is axes[1]

    # Implement a button to reset the motor position
    if data.buttons[0] == 1:  # Assuming the reset button is button 0
        motor_command.data = 0.0  # Send 0.0 to reset to 0-degree position

    pub.publish(motor_command)

if __name__ == '__main__':
    rospy.init_node('joy_to_motor')
    pub = rospy.Publisher('motor_command', Float32, queue_size=10)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.spin()
