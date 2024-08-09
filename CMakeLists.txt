cmake_minimum_required(VERSION 3.16)
project(uvic_rover VERSION 1.0.0 LANGUAGES CXX)

## Compile as C++17, supported in ROS Kinetic and newer, enable some static analysis
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ROVER_CPP_COMPILE_OPTIONS -Wall -Wextra -Werror -pedantic)

# ROS packages list
set(ROVER_PACKAGES
        rospy
        roscpp
        std_msgs
        message_generation
        sensor_msgs
        rosserial_arduino
        rosserial_client
        joy
        cv_bridge
        phidgets_api
        )

# Message files
set(ROVER_MESSAGE_FILES
        Arm.msg # not really using
        Speed.msg
        )


# Service files
# set(ROVER_SERVICE_FILES empty-for-now)


# Generate messages list
set(ROVER_ROS_MESSAGES
        sensor_msgs
        std_msgs
        )

# catkin packages list
set(ROVER_CATKIN_PACKAGES
  roscpp 
  rospy 
  std_msgs 
  message_runtime
)


macro(add_gazebo_plugin_macro name sources includes)
    add_cpp_library_macro(${name} ${sources} ${includes})

    # TODO: find a proper variable name that points to /opt/ros/noetic/lib
    target_link_directories(${name} PRIVATE ${GAZEBO_LIBRARY_DIRS} /opt/ros/noetic/lib)
    target_link_libraries(${name} PRIVATE ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
    target_include_directories(${name} SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
endmacro()


# launch install macro
macro(install_launch_macro)
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            )
endmacro()


# macro(add_tests_macro)
#     # Add C++ unit tests

#     # Python unit tests

#     # Integration tests (python and c++)
#     find_package(rostest REQUIRED)
# endmacro()


# Subdirectories before message declarations
set(CMAKE_SUBDIRS "")



# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Specify build details by appending to lists
# and implementing the some extra macros
# Add new devices as elseif blocks
# Select device with --cmake-flags -D DEVICE=<fill in device>
# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
if (${DEVICE} MATCHES "raspi4")
    # -=-=-=-=-
    # Lists
    # -=-=-=-=-

    # Add any raspi4 specific packages here
    # list(APPEND _PACKAGES )

    # -=-=-=-=-
    # Macros
    # -=-=-=-=-

    # macro(include_directories_macro)
    #     include_directories(
    #             ${catkin_INCLUDE_DIRS}
    #     )
    # endmacro()

    # # define an add and link macro
    # # Put items here to build
    # macro(add_and_link_macro)
    #     add_cpp_node_macro(brushed_motors "src/esw/brushed_motors/*")
    # endmacro()
else ()
    # -=-=-=-=-
    # Lists
    # -=-=-=-=-

    # Add any laptop specific packages here
    list(APPEND ROVER_PACKAGES
            tf2_geometry_msgs
            tf2_ros
            tf2
            visualization_msgs
            image_transport
            cv_bridge
            gazebo_ros
            camera_info_manager
            std_srvs
            )

    # append subdirectories
    # list(APPEND CMAKE_SUBDIRS
    #         starter_project/teleop
    #         )

    # -=-=-=-=-
    # Macros
    # -=-=-=-=-

    # These packages need to be found individually
    # macro(add_packages_macro)
    #     find_package(OpenCV REQUIRED COMPONENTS core aruco)
    #     find_package(gazebo REQUIRED)
    #     find_package(Eigen3 REQUIRED)
    #     find_package(ZED 2)
    #     if (ZED_FOUND)
    #         set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
    #         set(CMAKE_CUDA_STANDARD 17)
    #         set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    #         # Jetson Xavier NX is Volta 7.2 architecture
    #         # Perception Laptop (A4000, Quadro version of RTX 3080) is Ampere 8.6 architecture
    #         set(CMAKE_CUDA_ARCHITECTURES 72 86)
    #         enable_language(CUDA)
    #     endif ()
    # endmacro()

    # define an add and link macro
    # Put items here to build
    # macro(add_and_link_macro)
    #     add_cpp_nodelet_macro(tag_detector_nodelet "src/perception/tag_detector/*.cpp" "src/perception/tag_detector")
    #     target_link_libraries(tag_detector_nodelet PRIVATE ${OpenCV_LIBRARIES} lie)

    #     add_cpp_node_macro(brushed_motors "src/esw/brushed_motors/*")

    #     add_cpp_library_macro(lie "src/util/lie/*.cpp" "src/util/lie")

    #     add_gazebo_plugin_macro(differential_drive_plugin_6w "src/gazebo/differential_drive_6w.cpp" "src")

    #     add_gazebo_plugin_macro(kinect_plugin "src/gazebo/gazebo_ros_openni_kinect.cpp" "src/gazebo")
    #     target_link_libraries(kinect_plugin PRIVATE gazebo_ros_camera_utils DepthCameraPlugin Eigen3::Eigen)
    #     set_target_properties(kinect_plugin PROPERTIES CXX_CLANG_TIDY "")

    #     if (ZED_FOUND)
    #         #            add_cpp_node_macro(zed_node "src/perception/zed_wrapper/*.c*")
    #         #            target_link_options(zed_node PRIVATE "LINKER:--copy-dt-needed-entries")
    #         #            target_include_directories(zed_node SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    #         #            target_link_libraries(zed_node PRIVATE ${ZED_LIBRARIES} ${OpenCV_LIBRARIES} ${SPECIAL_OS_LIBS} lie tag_detector_nodelet)

    #         add_cpp_nodelet_macro(zed_nodelet "src/perception/zed_wrapper/*.c*" , "src/perception/zed_wrapper")
    #         target_include_directories(zed_nodelet SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    #         target_link_libraries(zed_nodelet PRIVATE ${ZED_LIBRARIES} ${OpenCV_LIBRARIES} ${SPECIAL_OS_LIBS} lie)
    #         target_compile_definitions(zed_nodelet PRIVATE ROVER_IS_NODELET)
    #         target_compile_options(zed_nodelet PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-extended-lambda>)
    #     endif ()
    # endmacro()
endif ()


# 3. Find Packages
find_package(
        catkin REQUIRED COMPONENTS
        ${ROVER_PACKAGES}
)

if (COMMAND add_packages_macro)
    add_packages_macro()
endif ()


# 4. Python module support
catkin_python_setup()


# 4.5. Subdirectories before message declarations
if (NOT "${CMAKE_SUBDIRS}" MATCHES "")
    add_subdirectory(
            ${CMAKE_SUBDIRS}
    )
endif ()


# 5. Message Generators (add_xxx)
add_message_files(
        FILES
        ${ROVER_MESSAGE_FILES}
)

# add_service_files(
#         FILES
#         ${ROVER_SERVICE_FILES}
# )


# 6. Invoke messages (generate_messages)
generate_messages(
        DEPENDENCIES
        ${ROVER_ROS_MESSAGES}
)

# generate_dynamic_reconfigure_options(
#         ${ROVER_PARAMETERS}
# )


# 7. Specify package build info export (catkin_package)
catkin_package(
        CATKIN_DEPENDS
        ${ROVER_CATKIN_PACKAGES}
)


if (COMMAND add_and_link_macro)
    add_and_link_macro()
endif ()


# 9. Tests to build
# if (COMMAND add_tests_macro)
#     add_tests_macro()
# endif ()


# 10. Install rules
install_launch_macro()
if (COMMAND additional_install_macro)
    additional_install_macro()
endif ()

# Install Python scripts I ADDED THIS (TEST)
catkin_install_python(PROGRAMS
  src/nav/gps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)