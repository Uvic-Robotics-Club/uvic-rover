#!/usr/bin/env python 
 # joystick_demo.py
# This program will read the x and y axis data from a USB connected joystick and
# will convert that data into left/right speed values for the runt rover. 
# This program then processes the left/right speed values and increments or decrements 
# them accordingly


from time import time
import rospy
from sensor_msgs.msg import Joy
from arm.msg import Arm
MAX_VAL = 255
X_AXIS_DEADZONE = 0.25 * MAX_VAL
Y_AXIS_DEADZONE = 0.25* MAX_VAL


pub = rospy.Publisher('armCommands', Arm, queue_size=10)

# max_PWM = 255

'''
logitech joystick axis

axis 1 ->  shoulder
axis 5 -> elbow

'''

def control_arm(sub):

    # init all buttons on logitech joystick
    values = [0,0,0,0,0,0,0]
    shoulder_axis = sub.axes[1] * MAX_VAL
    elbow_axis = sub.axes[5] * MAX_VAL
    wristY = [sub.buttons[2],sub.buttons[4]]
    wristX = [sub.buttons[5],sub.buttons[3]]
    gripper = [sub.buttons[0],sub.buttons[1]]
    base = [sub.buttons[11], sub.buttons[10]]
    stop_safety = sub.buttons[7]

    values[0] = 0
    if(shoulder_axis > 25.0):
        values[0] = 1
    elif(shoulder_axis < -25.0):
        values[0] = -1
    else:
        values[0] = 0

    values[1] = 0
    if(elbow_axis > 50.0):
        values[1] = 1
    elif(elbow_axis < -50.0):
        values[1] = -1
    else:
        values[1] = 0

    values[2] = 0
    if(wristX[0] == 1):
        values[2] = 1
    elif(wristX[1] == 1):
        values[2] = -1
    elif(wristX[0] == 0 and wristX == 0):
        values[2] = 0
    
    values[3] = 0
    if(wristY[0] == 1):
        values[3] = 1
    elif(wristY[1] == 1):
        values[3] = -1
    elif(wristY[0] == 0 and wristY[1] == 0):
        values[3] = 0

    values[4] = 0
    if(gripper[0] == 1):
        values[4] = 1
    elif(gripper[1] == 1):
        values[4] = -1
    elif(gripper[0] == 0 and gripper[1] == 0):
        values[4] = 0
    
    values[5] = 0
    if(base[0] == 1):
        values[5] = 1
    elif(base[1] == 1):
        values[5] = -1
    elif(base[0] == 0 and base[1] == 0):
        values[5] = 0


    # need to press the stop safety inorder to move the arm
    if(stop_safety == 0):
        values = [0,0,0,0,0,0,0]

    msg = Arm()
    msg.data = values
    pub.publish(msg)




# similar to map() in arduino
# remaps values from 0-100 to 0-255
def remap(OldValue,OldMin,OldMax,NewMin,NewMax):
    OldRange = OldMax - OldMin
    if(OldRange == 0):
        NewValue = NewMin
    else:
        NewRange = NewMax - NewMin
        NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    return NewValue

def main():
    rospy.init_node('joystick_demo', log_level=rospy.DEBUG)
    rospy.Subscriber('j1',Joy,control_arm)
    rospy.spin()
    
if __name__ == '__main__':
    main()