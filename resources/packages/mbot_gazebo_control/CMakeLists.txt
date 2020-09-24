cmake_minimum_required(VERSION 2.8.3)
project(mbot_gazebo_control)

find_package(catkin REQUIRED
    roscpp
    std_msgs
)

catkin_python_setup()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
)

### EXECUTABLES
add_executable(mbot_head_republisher ros/src/mbot_head_gazebo_republisher.cpp)
target_link_libraries(mbot_head_republisher ${catkin_LIBRARIES})
