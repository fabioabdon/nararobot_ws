cmake_minimum_required(VERSION 2.8.3)
project(nararobot)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries 
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf2
		tf2_ros
    nav_msgs 
    nara_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nararobot
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs nara_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nara_base src/nara_base.cpp)
add_executable(nara_base_node src/nara_base_node.cpp)
target_link_libraries(nara_base_node nara_base ${catkin_LIBRARIES})
add_dependencies(nara_base nara_msgs_generate_messages_cpp)


#############
## Install ##
#############

# Future work

#############
## Testing ##
#############

# Future work
