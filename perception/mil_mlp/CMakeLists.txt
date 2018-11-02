cmake_minimum_required(VERSION 2.8.3)
project(mil_mlp)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  INCLUDE_DIRS
    labelbox_data_processing
    docker_tf
  CATKIN_DEPENDS
    rospy
  DEPENDS
)


catkin_python_setup()

include_directories(
  docker_tf
  labelbox_data_processing
  ${catkin_INCLUDE_DIRS}
)
