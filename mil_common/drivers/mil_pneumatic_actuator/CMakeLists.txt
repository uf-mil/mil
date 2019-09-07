cmake_minimum_required(VERSION 2.8.3)
project(mil_pneumatic_actuator)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

catkin_python_setup()

add_service_files(
  FILES
  SetValve.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
