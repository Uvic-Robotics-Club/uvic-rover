# ROS-runt-rover
ROS implementation of runt rover

This assumes that you have Ubuntu 20.04, ROS Noetic with a catkin workspace, and rosserial installed. 

Clone this repository inside < your catkin workspace >/src 

Once cloned, As far as I know everytime a custom message is added, you need to run this command and remake the arduino libraries.
```
  cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
```

**Building:** 
If you are running this on the Rasberry Pi 3b+ 
use this command inside your catkin workspace 
```
catkin_make -j 1 -DCMAKE_CXX_FLAGS="--param ggc-min-expand=20"
```
otherwise just run ```catkin_make```

**Run file**
there is a launch file inside the launch directory, this will bootup the arduino and the joystick and output on the same terminal.

```roslaunch run_runt_rover.launch```

Note: make sure that the port for the arduino is correct and to use the old bootloader for the arduino nano. 
