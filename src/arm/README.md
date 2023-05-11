# Running Arm
Instructions here detail how to run the arm. This document will keep changing as the arm is being worked on.

Remember to source ros on a new terminal

## Boot up ROS
```
source devel/setup.bash
roscore 
```

## Setting up joystick
More information can be found here 
http://wiki.ros.org/joy

If this is your first time setting the joystick up then follow the link above to set permissions correctly

Get a list of input devices js0 would be joystick
```
ls /dev/input/
```

tell joy package which device we are using (X should be a digit)
```
rosparam set joy_node/dev "/dev/input/jsX"
```
Run the node
```
rosrun joy joy_node
```

On another terminal, this displays topic data from joystick
```
rostopic echo joy
```

## Arduinos


Run rosserial 

Check out launch folder when launching multiple ardunios


This command below executes the code on the arduino on port /ttyUSB0
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```