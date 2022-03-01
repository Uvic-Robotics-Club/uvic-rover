# ROS-runt-rover
ROS implementation of runt rover

This assumes that you have Ubuntu 20.04, ROS Noetic with a catkin workspace, and rosserial installed. 

Clone this repository inside < your catkin workspace >/src 

**Setting up Rosserial:**
Need to run this command to make the specific library with the custom message.
Check the example in this link for the correct arguements
[link to this](http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages)

```
rosrun rosserial_client make_library.py path_to_libraries your_message_package
```

Once above command is run, As far as I know everytime a custom message is added, you need to run this command and remake the arduino libraries.
```
  cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
```
Note: make sure that the port for the arduino is correct and to use the old bootloader for the arduino nano. 

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
