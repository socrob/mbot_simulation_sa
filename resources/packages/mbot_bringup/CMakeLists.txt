cmake_minimum_required(VERSION 2.8.3)
project(mbot_bringup)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY ros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_python_setup()
