#!/usr/bin/env python 
 # joystick_demo.py
# This program will read the x and y axis data from a USB connected joystick and
# will convert that data into left/right speed values for the runt rover. 
# This program then processes the left/right speed values and increments or decrements 
# them accordingly


import pygame
from time import time
import rospy
from runt_rover.msg import DriveTrain

MAX_VAL = 100
X_AXIS_DEADZONE = 0.14 * MAX_VAL
Y_AXIS_DEADZONE = 0.14 * MAX_VAL
Z_AXIS_DEADZONE = 0.20 * MAX_VAL # Rotation around z axis
SLIDER_OFFSET = 1 * MAX_VAL # Slider in front of joystick




class joystick_demo:

    def control_runt_rover(self):

        # Create an Speed Publisher joystick data
        pub = rospy.Publisher('drive_train',DriveTrain,queue_size=10)
        rospy.init_node('joystick_demo')
        # 15 is in Hz, I have found this value through trial and error
        # Anything that is too low can overload the arduino buffer and cause it 
        # to lag and do weird memory stuff
        rate = rospy.Rate(15) 
        
        pygame.init()
        current_time = time()


        # Initialize joystick
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            print("Cannot find joystick. Not running joystick.")
            return

        while pygame.joystick.get_count() > 0 and not rospy.is_shutdown():
            if (time() - current_time) < 0.05:
                continue
            current_time = time()

            # Retrieve joystick data
            # X and Y axis is range [-100.0, 100.0] where negative is reverse
            pygame.event.get()
            x_axis = joystick.get_axis(0) * MAX_VAL
            y_axis = (0-joystick.get_axis(1))* MAX_VAL

            # Rotation around Z axis is range [-100.0, 100.0] where negative is left rotation
            z_axis = joystick.get_axis(3)* MAX_VAL
            # print(x_axis, y_axis, z_axis)

            
            # Limiter is in range [0, 10.0] where 0 is when slider is all the way
            # back towards negative sign
            limiter = (0 - joystick.get_axis(2)) * MAX_VAL
            limiter = (limiter + SLIDER_OFFSET) / 2 # Maps from [-100.0, 100.0] to [0, 100.0]

            # Set initial speed before considering turning
            speedLeft = speedRight = y_axis

            if abs(x_axis) < X_AXIS_DEADZONE and abs(y_axis) < Y_AXIS_DEADZONE:
                # Joystick is centred
                if abs(z_axis) > Z_AXIS_DEADZONE:
                    # Rotate rover in place
                    speedLeft += z_axis
                    speedRight -= z_axis
            else:
                # Joystick is not centred
                if x_axis != 0:
                    # Turn rover
                    speedLeft += x_axis
                    speedRight -= x_axis

            # Limit values
            speedLeft = -limiter if speedLeft < -limiter else limiter if speedLeft > limiter else speedLeft
            speedRight = -limiter if speedRight < -limiter else limiter if speedRight > limiter else speedRight

            # Arduino expects ints
            speed_left = int(speedLeft)
            speed_right = int(speedRight)

            speed_left_setpoint = 0
            speed_right_setpoint = 0

            if(speed_left < speed_left_setpoint):
                speed_left+=1
            elif(speed_left>speed_left_setpoint):
                speed_left-=1
            
            if(speed_right < speed_right_setpoint):
                speed_right+=1
            elif(speed_right>speed_right_setpoint):
                speed_right-=1


            write_direction_left = 0 
            write_direction_right = 0
            write_speed_left = 0
            write_speed_right = 0
            duty_cycle_max = 255
        
            # reverse is 0
            # forward is 1

            if(speed_left>0):
                write_direction_left = 0
            else:
                write_direction_left = 1
            
            if(speed_right>0):
                write_direction_right = 0
            else:
                write_direction_right = 1

            write_speed_left = joystick_demo.remap(abs(speed_left),0,100,0,255)
            write_speed_right = joystick_demo.remap(abs(speed_right),0,100,0,255)



            msg = DriveTrain()
            msg.rightspeed = int(write_speed_right)
            msg.leftspeed = int(write_speed_left)
            msg.rightdirection = write_direction_right
            msg.leftdirection = write_direction_left

            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()
            # rospy.loginfo(msg)
            # pub.publish(msg)
    

    # similar to map() in arduino
    # remaps values from 0-100 to 0-255
    @staticmethod
    def remap(OldValue,OldMin,OldMax,NewMin,NewMax):
        OldRange = OldMax - OldMin
        if(OldRange == 0):
            NewValue = NewMin
        else:
            NewRange = NewMax - NewMin
            NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
        return NewValue

def main():
    joystick_demo().control_runt_rover()