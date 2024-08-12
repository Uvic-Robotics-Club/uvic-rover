#!/usr/bin/env python

import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
from sensor_msgs.msg import Joy


# Return BLDC motor phidget object
def initialize_motor(serial_number, hub_port):
    motor = BLDCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setHubPort(hub_port)
    motor.setIsHubPortDevice(False)
    motor.openWaitForAttachment(5000)
    return motor


vint_hub_serial_number = 757580  # Replace with your VINT Hub's serial number


motors = {
    'back_left': initialize_motor(vint_hub_serial_number, 5),
    'back_right': initialize_motor(vint_hub_serial_number, 0),
    'front_left': initialize_motor(vint_hub_serial_number, 4),
    'front_right': initialize_motor(vint_hub_serial_number, 3)
}


# Callback function for joy messages
def joy_callback(data):


    if data.buttons[0] == 1:
        motors['back_left'].setTargetVelocity(0)
        motors['back_right'].setTargetVelocity(0)
        motors['front_left'].setTargetVelocity(0)
        motors['front_right'].setTargetVelocity(0)
        return
    else:
        motors['back_left'].setTargetVelocity(data.axes[4])
        motors['back_right'].setTargetVelocity(-data.axes[4])
        motors['front_left'].setTargetVelocity(-data.axes[4])
        motors['front_right'].setTargetVelocity(data.axes[4])

def main():

    rospy.init_node('phidget_motor_control')
    rate = rospy.Rate(15) 
    rospy.Subscriber('joy', Joy, joy_callback, queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    main()
