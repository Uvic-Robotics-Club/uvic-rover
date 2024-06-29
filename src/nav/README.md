## NAVIGATIONAL SYSTEMS GUIDE

This guide will explain the step-by-step process on how to activate the rover's GPS and IMU, setup a mapproxy docker container to acquire and cache map tiles from Google Maps, and display his data through ROS's MapViz program.

NOTE: Remember to start an roscore instance and source the catkin workspace's devel/setup.bash file prior to inputting these commands.

## STEP #1: Starting up the GPS and IMU

    cd ~/catkin_ws
    rosrun uvic_rover gps.py

FURTHER INSTRUCTIONS TO BE ADDED

## STEP #2: Creating the Mapproxy Docker Container

    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

OPTIONAL: To verify that the docker container was successfully created, use the following to list all running containers.

    sudo docker ps

## STEP #3: Opening MapViz

    roslaunch mapviz mapviz.launch


