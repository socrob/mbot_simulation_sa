cmake_minimum_required(VERSION 2.8.3)
project(mbot_odom_gt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  std_msgs
  tf
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    std_msgs
    tf
  LIBRARIES mbot_gazebo_ros_p3d
)

add_library(mbot_gazebo_ros_p3d src/gazebo_ros_p3d.cpp)
target_link_libraries(mbot_gazebo_ros_p3d ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
