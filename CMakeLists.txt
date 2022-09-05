cmake_minimum_required(VERSION 3.0.2)
project(hsr_small_objects)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
std_msgs
rospy
roscpp
genmsg 
actionlib_msgs
actionlib
geometry_msgs
sensor_msgs)


add_action_files(DIRECTORY action 
FILES FindObject.action ArmMovement.action)

generate_messages(DEPENDENCIES 
actionlib_msgs
geometry_msgs
std_msgs)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HSR_Test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#catkin_install_python(PROGRAMS scripts/arm_movement_action_server.py scripts/find_object_action_server.py #scripts/small_objects_statemachine.py scripts/user_input.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



