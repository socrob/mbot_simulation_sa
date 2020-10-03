cmake_minimum_required(VERSION 2.8.3)
project(mbot_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

### INSTALLS
# to make the install effective you need to run:
#    catkin build --this --make-args install

# the place where gazebo puts the models by default
set(GAZEBO_MODEL_PATH $ENV{HOME}/.gazebo/models)

# create folder where meshes are going to be installed
add_custom_target(COMMAND cmake -E make_directory ${GAZEBO_MODEL_PATH}/${PROJECT_NAME})

# copy meshes to gazebo models path
install(DIRECTORY meshes
  DESTINATION ${GAZEBO_MODEL_PATH}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.dae"
  PATTERN "*.png"
)
