cmake_minimum_required(VERSION 2.8.3)
project(navigator_shooter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
  navigator_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)
rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)
add_dependencies(${PROJECT_NAME}_ros_lib navigator_msgs_generate_messages)
add_dependencies(${PROJECT_NAME}_ros_lib ${catkin_EXPORTED_TARGETS})
#rosserial_add_client_target(firmware arduino ALL)
#rosserial_add_client_target(firmware arduino-upload)
add_dependencies(${PROJECT_NAME}_firmware ${PROJECT_NAME}_ros_lib)
