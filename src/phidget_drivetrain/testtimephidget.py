#!/usr/bin/env python

import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
from sensor_msgs.msg import Joy

"""Previous simple drive code"""



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
    # 'back_right': initialize_motor(vint_hub_serial_number, 0),
    # 'front_left': initialize_motor(vint_hub_serial_number, 4),
    # 'front_right': initialize_motor(vint_hub_serial_number, 3)
}


# Callback function for joy messages
def joy_callback(data):
    # Use the vertical axis of the left stick for forward/backward control
    forward = data.axes[4]  # Assuming left stick vertical axis is at index 1
    stopbutton = data.buttons[1]
    # print(stopbutton)
    # if stopbutton == 1:
    #     motors['back_left'].setTargetVelocity(motors['back_left'].getMinVelocity())
    #     motors['back_right'].setTargetVelocity(motors['back_left'].getMinVelocity())
    #     motors['front_left'].setTargetVelocity(motors['back_left'].getMinVelocity())
    #     motors['front_right'].setTargetVelocity(motors['back_left'].getMinVelocity())
    #     return 
    # Set all motors to the same velocity for forward/backward movement
    print("data:" ,data)
    # print("forward:",forward)

    motors['back_left'].setTargetVelocity(0.5)

     
    # motors['back_right'].setTargetVelocity(-forward)
    # motors['front_left'].setTargetVelocity(-forward)
    # motors['front_right'].setTargetVelocity(forward)

def main():
    # global motors

    # Define the serial number of your VINT Hub
    # vint_hub_serial_number = 757580  # Replace with your VINT Hub's serial number

    # Initialize motors
    # Initialize ROS node and subscribe to joy topic
    rospy.init_node('phidget_motor_control')
    # rate = rospy.Rate(15) 
    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.spin()

    # Close motors when done
    # for motor in motors.values():
    #     motor.close()

if __name__ == '__main__':
    main()
