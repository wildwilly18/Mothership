cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

## Declare Gazebo Stuff
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

## Declare Catkin stuff aka ROS
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(model_stationary SHARED model_stationary.cc)
target_link_libraries(model_stationary ${GAZEBO_LIBRARIES})
