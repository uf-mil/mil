cmake_minimum_required(VERSION 3.0.2)
project(subjugator_simulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  message_generation
  message_runtime
)
catkin_python_setup()

add_service_files(
  FILES
  SimSetPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
   LIBRARIES subjugator_simulation
   CATKINs_DEPENDS rospy geometry_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
