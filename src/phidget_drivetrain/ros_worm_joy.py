#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool

class WheelController:
    def __init__(self):
        rospy.init_node('wheel_controller')
        
        self.motor_pub = rospy.Publisher('motor_commands', Float32MultiArray, queue_size=10)
        self.home_pub = rospy.Publisher('home_command', Bool, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.motor_cmd = Float32MultiArray()
        self.motor_cmd.data = [0.0, 0.0, 0.0]
        
    def joy_callback(self, joy_msg):
        # Assuming axes[0] and axes[1] control the first two motors
        # and axes[3] and axes[4] control the other two motors
        self.motor_cmd.data[0] = joy_msg.axes[0]
        self.motor_cmd.data[1] = joy_msg.axes[0]
        
        # Publish motor commands
        self.motor_pub.publish(self.motor_cmd)
        
        # Check if home button is pressed (assuming it's button 2)
        if joy_msg.buttons[2] == 1:
            self.home_pub.publish(Bool(True))
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    controller = WheelController()
    controller.run()