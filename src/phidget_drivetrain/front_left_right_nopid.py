#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Bool

class JoystickMotorControl:
    def __init__(self):
        rospy.init_node('joystick_motor_control', anonymous=True)
        
        # Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        # Publishers for left and right motors
        self.pub_left_pwm = rospy.Publisher('worm_gear_front_left_pwm', Int16, queue_size=10)
        self.pub_left_dir = rospy.Publisher('worm_gear_front_left_dir', Bool, queue_size=10)
        self.pub_right_pwm = rospy.Publisher('worm_gear_front_right_pwm', Int16, queue_size=10)
        self.pub_right_dir = rospy.Publisher('worm_gear_front_right_dir', Bool, queue_size=10)
        
        self.pub_back_pwm = rospy.Publisher('worm_gear_back_right_pwm', Int16, queue_size=10)
        self.pub_back_dir = rospy.Publisher('worm_gear_back_right_dir', Bool, queue_size=10)
        
        # Assume left stick Y-axis is at index 1, adjust if needed
        self.axis_left_y = 0
        
        rospy.spin()

    def joy_callback(self, data):
        # Get joystick value (-1 to 1)
        joy_value = data.axes[self.axis_left_y]
        
        # Convert joystick value to PWM (0 to 255)
        pwm_value = int(abs(joy_value) * 100)
        
        # Determine direction
        direction = False
        direction = joy_value >= 0
        
        # Publish PWM and direction for both motors
        self.pub_left_pwm.publish(pwm_value)
        self.pub_left_dir.publish(direction)
        self.pub_right_pwm.publish(pwm_value)
        self.pub_right_dir.publish(direction)

        self.pub_back_pwm.publish(pwm_value)
        self.pub_back_dir.publish(direction)

if __name__ == '__main__':
    try:
        JoystickMotorControl()
    except rospy.ROSInterruptException:
        pass