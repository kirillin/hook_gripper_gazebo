cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(hook_gripper_gazebo)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  # gazebo_dev
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/gazebo_ros_hook_gripper.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${catkin_INCLUDE_DIRS})