#!/usr/bin/env python

import numpy as np
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def IK(p_end):
    # knowns
    d1 = 77.5
    d2 = 89.5
    d3 = 74.5
    d4 = 276.5
    d_ee = 265
    l2 = 320

    # Rotation matrices
    Rz = np.array([
        [np.cos(p_end[3]), -np.sin(p_end[3]), 0],
        [np.sin(p_end[3]), np.cos(p_end[3]), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(p_end[4]), 0, np.sin(p_end[4])],
        [0, 1, 0],
        [-np.sin(p_end[4]), 0, np.cos(p_end[4])]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(p_end[5]), -np.sin(p_end[5])],
        [0, np.sin(p_end[5]), np.cos(p_end[5])]
    ])

    R06 = Rz @ Ry @ Rx

    p_x = p_end[0]
    p_y = p_end[1]
    p_z = p_end[2]

    # theta1 calculation
    theta1 = np.arctan2(-p_x, p_y) + np.arctan2(np.sqrt(abs(p_y**2 + p_x**2 - (d2 - d3)**2)), (d2 - d3))
    if abs(theta1) < 0.00872665:
        theta1 = 0

    # a calculation for theta3
    a = (d1**2 - 2*d1*p_z + p_x**2 + p_y**2 + p_z**2 - d2**2 + 2*d2*d3 - d3**2 - d4**2 - l2**2) / (2 * d4 * l2)
    theta3 = np.arctan2(a, np.sqrt(abs(1 - a**2)))
    if abs(theta3) < 0.00872665:
        theta3 = 0
    
    #change theta3 due to new configuration
    theta3=theta3+math.pi
    
    # Matrix C and IK_s1
    C = np.array([
        [l2 + d4 * np.sin(theta3), d4 * np.cos(theta3)],
        [d4 * np.cos(theta3), -d4 * np.sin(theta3) - l2]
    ])
    
    IK_s1 = np.array([
        p_x * np.cos(theta1) + p_y * np.sin(theta1),
        p_y * np.cos(theta1) - p_x * np.sin(theta1),
        p_z - d1
    ])

    # theta2 calculation
    a = C[1, 0]
    b = C[0, 0]
    c = IK_s1[2]
    d = IK_s1[0]
    theta2 = np.arctan2(a * d - b * c, a * c + b * d)
    if abs(theta2) < 0.00872665:
        theta2 = 0

    #change theta2 due to new configuration
    theta2=theta2-math.pi/2

    # R03 matrix
    R03 = np.array([
        [np.cos(theta1) * np.cos(theta2) * np.cos(theta3) - np.cos(theta1) * np.sin(theta2) * np.sin(theta3), -np.cos(theta1) * np.cos(theta2) * np.sin(theta3) - np.cos(theta1) * np.cos(theta3) * np.sin(theta2), -np.sin(theta1)],
        [np.cos(theta2) * np.cos(theta3) * np.sin(theta1) - np.sin(theta1) * np.sin(theta2) * np.sin(theta3), -np.cos(theta2) * np.sin(theta1) * np.sin(theta3) - np.cos(theta3) * np.sin(theta1) * np.sin(theta2), np.cos(theta1)],
        [-np.cos(theta2) * np.sin(theta3) - np.cos(theta3) * np.sin(theta2), np.sin(theta2) * np.sin(theta3) - np.cos(theta2) * np.cos(theta3), 0]
    ])

    R36 = R03.T @ R06

    r11, r12, r13 = R36[0, :]
    r21, r22, r23 = R36[1, :]
    r31, r32, r33 = R36[2, :]

    # theta6 calculation
    theta6 = np.arctan2(-r22, r21)
    if abs(theta6) < 0.00872665:
        theta6 = 0

    # theta5 calculation
    theta5 = np.arctan2(r21 * np.cos(theta6) - r22 * np.sin(theta6), -r23)
    if abs(theta5) < 0.00872665:
        theta5 = 0

    #change theta5 due to new configuration
    theta5=theta5-math.pi/2

    # theta4 calculation
    theta4 = np.arctan2(-(r12 * np.cos(theta6) + r11 * np.sin(theta6)), r32 * np.cos(theta6) + r31 * np.sin(theta6))
    if abs(theta4) < 0.00872665:
        theta4 = 0

    joints = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    return joints



class JoyIKNode:
    def __init__(self):
        rospy.init_node('joy_ik_node', anonymous=True)
        
        # Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        # Create a publisher for String messages
        self.motor0 = rospy.Publisher("motor0", String, queue_size=10)
        self.motor1 = rospy.Publisher("motor1", String, queue_size=10)
        self.motor2 = rospy.Publisher("motor2", String, queue_size=10)
        self.motor3 = rospy.Publisher("motor3", String, queue_size=10)
        self.motor4 = rospy.Publisher("motor4", String, queue_size=10)
        self.motor5 = rospy.Publisher("motor5", String, queue_size=10)

        # Set the loop rate (10 Hz)
        self.rate = rospy.Rate(5)
        
        # Initialize end-effector position and orientation
        self.p_end = np.array([276.5, 15.0, 397.5, 0.0, 0.0, 0.0])
        
        # Set step sizes for position and orientation changes
        self.pos_step = 1  # 1 mm
        self.rot_step = 2*(math.pi/180)  # radians
        rospy.spin()
        
    def joy_callback(self, data):
        # Update end-effector position and orientation based on joystick input
        # Left stick: X and Y position
        self.p_end[0] += data.axes[1] * self.pos_step/10
        self.p_end[1] += data.axes[0] * self.pos_step/10
        
        # # Right stick: Z position and rotation around Z
        self.p_end[2] -= data.axes[3] * self.pos_step

        #roll
        self.p_end[3] -= data.axes[6] * self.rot_step
        self.p_end[4] += data.buttons[0] * self.rot_step
        if data.buttons[3] == 1:
            self.p_end[4] -= data.buttons[3] * self.rot_step
        if data.buttons[0] ==1:
            self.p_end[4] += data.buttons[0] * self.rot_step
        
        if data.buttons[5] == 1:
            self.p_end[5] += data.buttons[5] * self.rot_step
        if data.buttons[4] == 1:
            self.p_end[5] -= data.buttons[4] * self.rot_step
        # if data.buttons[]

        # # Buttons: rotation around X and Y
        # if data.buttons[4]:  # Left bumper
        #     self.p_end[4] += self.rot_step
        # if data.buttons[5]:  # Right bumper
        #     self.p_end[4] -= self.rot_step
        # if data.buttons[6]:  # Left trigger
        #     self.p_end[5] += self.rot_step
        # if data.buttons[7]:  # Right trigger
        #     self.p_end[5] -= self.rot_step
        
        # # Call IK function
        # try:
        joints = IK(self.p_end)
        
        # print(self.p_end)
        # print(joints)
        #     # Create a string message with the joint angles
        print(joints)
        id_angle_string = []
        for i, angle in enumerate(joints):
            msg = String()
            msg.data = "{} {}".format(i,angle)
            id_angle_string.append(msg.data)   
        # print(id_angle_string)
        # self.motor0.publish(id_angle_string[0])        
        # self.motor1.publish(id_angle_string[1])        
        # self.motor2.publish(id_angle_string[2])        
        # self.motor3.publish(id_angle_string[3])        
        # self.motor4.publish(id_angle_string[4])        
        # self.motor5.publish(id_angle_string[5])        




        



if __name__ == '__main__':
    try:
        node = JoyIKNode()
    except rospy.ROSInterruptException:
        pass 