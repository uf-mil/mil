cmake_minimum_required(VERSION 2.8.3)
project(mil_poi)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  message_generation
  rospy
  geometry_msgs
)

catkin_python_setup()

add_message_files(
  FILES
    POI.msg
    POIArray.msg
)

add_service_files(
  FILES
    AddPOI.srv
    DeletePOI.srv
    MovePOI.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package()
