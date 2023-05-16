# ROS-runt-rover
ROS implementation of runt rover

This assumes that you have Ubuntu 20.04, ROS Noetic and rosserial installed. 

Clone this repository inside < your catkin workspace > 

The repo is our catkin_ws, so I recommend cloning then renaming this repo as 'src', then running catkin_make


```
git clone <url> src 
cd ROS-rover
catkin_make
```
If this is your first time building it, it will fail because some packages need to be installed and rosserial needs to be setup.

**Downloading packages**

For example you want to run the arm, and you want to install the joy package.
```
sudo apt-get install ros-noetic-<package-name>
sudo apt-get install ros-noetic-joy
```

**Setting up Rosserial:**

Follow this installation process http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Need to run this command to make the specific library with the custom message.
Check the example in this link for the correct arguements
(http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages). 

This assumes that you have rosserial already installed but not have built our custom message package (rover)

First we need to build our catkin workspace

## Creating new package
Simply create a new folder with your package name on the same level as the drivetrain folder. From here you will create a CMakeLists.txt and package.xml file and populate these accordingly, use previous CMakelists as a template but make sure to add in your dependencies.  

## Custom message in rosserial

Follow this tutorial (steps 1 and 2 and 5)
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Common_step_for_msg_and_srv

After this you need to go back into catkin_ws and put in
```
source devel/setup.bash 
rosrun rosserial_client make_library.py path_to_libraries
```
Note: idk why but this is very flaky, I find that you need to type in the full directory path for the libraries and we are leaving 
the argument after that blank so that it can build all the custom messages in our workspace

If this is the first time doing this, there should be an output and your package and msg should be listed

## Diagram of 2023 Rover 
Below is the minimum viable implementation of the rover

![odyssey2023 drawio](https://github.com/Uvic-Robotics-Club/ROS-rover/assets/42494981/f1491431-38e9-40fb-b39e-d3e123ff292f)


