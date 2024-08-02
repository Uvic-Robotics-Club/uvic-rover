#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from Phidget22.Devices.BLDCMotor import *

class QuadMotorController:
    def __init__(self):
        rospy.init_node('Phidget_motor_controller')
        
        # Initialize motors
        self.motors = []
        serial_numbers = [123456, 123457, 123458, 123459]  # Replace with your actual serial numbers
        for i, sn in enumerate(serial_numbers):
            motor = BLDCMotor()
            motor.setDeviceSerialNumber(sn)
            motor.setChannel(0)
            motor.openWaitForAttachment(5000)
            self.motors.append(motor)
        
        # Set up parameters
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.drive_mode = rospy.get_param('~drive_mode', 'tank')
        
        # Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
    def joy_callback(self, data):
        
        self.drive_mode = rospy.get_param('~drive_mode', 'tank')
        if data.buttons[0] == 1:
            self.drive_mode = rospy.set_param('~drive_mode', 'worm')
        else if data.buttons[1] == 1:
            self.drive_mode = rospy.set_param('~drive_mode', 'tank')

        if self.drive_mode == 'tank':
            self.tank_drive(data)
        else if self.drive_mode == 'worm':
            self.worm_drive(data)

    
    def worm_drive(self, data):
        # Get joystick value for forward/backward movement
        move = data.axes[4]  # Up/Down Axis stick right

        # Calculate motor speed
        velocity = move * self.max_velocity

        # Set motor velocities
        self.set_motor_velocity(0, velocity)
        self.set_motor_velocity(1, velocity)
        self.set_motor_velocity(2, velocity)
        self.set_motor_velocity(3, velocity)

    def tank_drive(self, data):
        # Get joystick values
        move = data.axes[4]  # Up/Down Axis stick right
        turn = data.axes[3]  # Left/Right Axis stick right

        # Calculate motor speeds
        left_velocity = (move + turn) * self.max_velocity
        right_velocity = (move - turn) * self.max_velocity

        # Set motor velocities
        self.set_motor_velocity(0, left_velocity)
        self.set_motor_velocity(1, left_velocity)
        self.set_motor_velocity(2, right_velocity)
        self.set_motor_velocity(3, right_velocity)
    
    def set_motor_velocity(self, motor_index, velocity):
        motor = self.motors[motor_index]
        motor.setTargetVelocity(velocity)
    
    def run(self):
        rospy.spin()
    
    def shutdown(self):
        for motor in self.motors:
            motor.setTargetVelocity(0)
            motor.close()

if __name__ == '__main__':
    controller = QuadMotorController()
    rospy.on_shutdown(controller.shutdown)
    controller.run()