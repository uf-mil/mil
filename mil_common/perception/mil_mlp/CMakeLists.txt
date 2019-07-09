cmake_minimum_required(VERSION 2.8.3)
project(mil_mlp)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  INCLUDE_DIRS
    ldp
    docker_tf
  CATKIN_DEPENDS
    rospy
  DEPENDS
)


catkin_python_setup()

