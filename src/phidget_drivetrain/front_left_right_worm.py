#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Bool, Int32
from simple_pid import PID

class JoystickMotorControl:
    def __init__(self):
        rospy.init_node('joystick_motor_control', anonymous=True)
        
        # Constants
        self.GEAR_RATIO = 1/522
        self.PPR = 11
        self.COUNTS_PER_REV = self.PPR / self.GEAR_RATIO

        # PID controllers
        self.pid_left = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
        self.pid_right = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
        
        # Subscribe to joy and encoder topics
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("left_encoder", Int32, self.left_encoder_callback)
        rospy.Subscriber("right_encoder", Int32, self.right_encoder_callback)
        
        # Publishers for left and right motors
        self.pub_left_pwm = rospy.Publisher('worm_gear_front_left_pwm', Int16, queue_size=10)
        self.pub_left_dir = rospy.Publisher('worm_gear_front_left_dir', Bool, queue_size=10)
        self.pub_right_pwm = rospy.Publisher('worm_gear_front_right_pwm', Int16, queue_size=10)
        self.pub_right_dir = rospy.Publisher('worm_gear_front_right_dir', Bool, queue_size=10)
        
        # Assume left stick Y-axis is at index 1, adjust if needed
        self.axis_left_y = 0
        
        # Motor state
        self.left_encoder_value = 0
        self.right_encoder_value = 0
        self.target_position = 0
        
        # Timer for PID control loop
        rospy.Timer(rospy.Duration(0.01), self.pid_control_loop)  # 100Hz control loop
        
        rospy.spin()

    def joy_callback(self, data):
        # Get joystick value (-1 to 1)
        joy_value = data.axes[self.axis_left_y]
        
        # Convert joystick value to target position (in encoder counts)
        self.target_position = joy_value * self.COUNTS_PER_REV
        
        # Update PID setpoints
        self.pid_left.setpoint = self.target_position
        self.pid_right.setpoint = self.target_position

    def left_encoder_callback(self, data):
        self.left_encoder_value = data.data

    def right_encoder_callback(self, data):
        self.right_encoder_value = data.data

    def pid_control_loop(self, event):
        # Compute PID output
        left_output = self.pid_left(self.left_encoder_value)
        right_output = self.pid_right(self.right_encoder_value)
        
        # Convert PID output to PWM and direction
        left_pwm = min(abs(int(left_output)), 255)
        right_pwm = min(abs(int(right_output)), 255)
        left_dir = left_output >= 0
        right_dir = right_output >= 0
        
        # Publish PWM and direction for both motors
        self.pub_left_pwm.publish(left_pwm)
        self.pub_left_dir.publish(left_dir)
        self.pub_right_pwm.publish(right_pwm)
        self.pub_right_dir.publish(right_dir)

if __name__ == '__main__':
    try:
        JoystickMotorControl()
    except rospy.ROSInterruptException:
        pass