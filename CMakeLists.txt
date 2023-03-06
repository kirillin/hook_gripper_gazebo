cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(hook_gripper_gazebo)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  # gazebo_dev
  # roscpp
  # std_msgs
)

# find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

# find_package(protobuf REQUIRED)
# set(PROTOBUF_IMPORT_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-11$")
#     set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#   endif()
# endforeach()

# find_package(Protobuf REQUIRED)
# pkg_check_modules(PROTOBUF protobuf)

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/gazebo_ros_hook_gripper.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})