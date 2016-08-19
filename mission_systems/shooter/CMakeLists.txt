cmake_minimum_required(VERSION 2.8.3)
project(navigator_shooter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
  navigator_msgs
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)
add_dependencies(${PROJECT_NAME}_ros_lib navigator_msgs_gencpp)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware arduino ALL)
rosserial_add_client_target(firmware arduino-upload)
add_dependencies(${PROJECT_NAME}_firmware_arduino ${PROJECT_NAME}_ros_lib)
