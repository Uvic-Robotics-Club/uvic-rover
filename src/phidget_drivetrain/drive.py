#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from phidgets_api.brushless_dc_motor import BrushlessDCMotor

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller')
        
        # Initialize the Brushless DC Motor
        self.motor = BrushlessDCMotor()
        self.motor.open(serial_number=0)  # Replace 0 with your Phidget's serial number
        
        # Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        # Set up parameters
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.axis_linear = rospy.get_param('~axis_linear', 1)  # Typically left stick vertical
        
    def joy_callback(self, data):
        # Get the joystick value (-1 to 1)
        joy_value = data.axes[self.axis_linear]
        
        # Map joystick value to motor velocity
        velocity = joy_value * self.max_velocity
        
        # Set the motor velocity
        self.motor.set_velocity_limit(abs(velocity))
        if velocity > 0:
            self.motor.set_target_velocity(self.motor.get_velocity_limit())
        elif velocity < 0:
            self.motor.set_target_velocity(-self.motor.get_velocity_limit())
        else:
            self.motor.set_target_velocity(0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = MotorController()
    controller.run()