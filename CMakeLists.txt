cmake_minimum_required(VERSION 3.0.2)
project(ishb_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mavros_msgs
  quadrotor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy mavros_msgs quadrotor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(super_mavros_bridge src/super_mavros_bridge.cpp)

## Add cmake target dependencies of the executable
add_dependencies(super_mavros_bridge ${catkin_EXPORTED_TARGETS})

## Specify libraries to link the executable against
target_link_libraries(super_mavros_bridge
  ${catkin_LIBRARIES}
)
