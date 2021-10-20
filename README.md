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

After plugging in the GPS and before launching ROS nodes, to ensure the GPS node works properly, you must disable the `gpsd.socket` service that is by default enabled and running. Furthermore, we must manually start `gpsd` and point it to the port where GPS data is arriving to such that we can display it. The following commands need to be run only once unless you restart the VM/computer:

```
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock
```

there is a launch file inside the launch directory, this will bootup the arduino and the joystick and output on the same terminal.

```roslaunch run_runt_rover.launch```

Note: make sure that the port for the arduino is correct and to use the old bootloader for the arduino nano. 
