cmake_minimum_required(VERSION 2.8.3)
project(monarch_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation geometry_msgs sensor_msgs std_msgs actionlib_msgs move_base_msgs)

add_message_files(
    FILES
    HeadControlSemantic.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

