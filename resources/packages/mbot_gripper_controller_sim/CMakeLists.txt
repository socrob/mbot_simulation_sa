cmake_minimum_required(VERSION 2.8.3)
project(mbot_gripper_controller_sim)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    mcr_manipulation_msgs
    roscpp
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    mcr_manipulation_msgs
    sensor_msgs
)

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_executable(sim_gripper_controller_node ros/src/sim_gripper_controller.cpp)

target_link_libraries(sim_gripper_controller_node ${catkin_LIBRARIES})
