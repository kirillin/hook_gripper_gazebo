cmake_minimum_required(VERSION 3.0.0)
project(hook_gripper_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gazebo_ros_hook_gripper.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})