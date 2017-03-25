cmake_minimum_required(VERSION 2.8.3)
project(mil_ros_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  
)
add_service_files(
  DIRECTORY srv
  FILES
  BaggerCommands.srv
)

generate_messages()
catkin_package()

