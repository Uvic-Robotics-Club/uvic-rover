cmake_minimum_required(VERSION 3.0.2)

include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)

generate_arduino_firmware(runt_rover
  SRCS ROS_runt_demo.ino ${ROS_LIB_DIR}/time.cpp
  BOARD NANO
  PORT /dev/ttyUSB0
)