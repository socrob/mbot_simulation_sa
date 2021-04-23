cmake_minimum_required(VERSION 2.8.3)
project(mbot_semantic_head_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  monarch_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
  roscpp
  monarch_msgs
)

install(DIRECTORY ros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

### EXECUTABLES
add_executable(semantic_head_controller ros/src/semantic_head_controller_server.cpp)
target_link_libraries(semantic_head_controller ${catkin_LIBRARIES})

