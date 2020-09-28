cmake_minimum_required(VERSION 3.0.2)
project(indyav_electrical)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  SteeringPotStamped.msg
  ThrottleBrakeStamped.msg
  WheelEncodersStamped.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES indyav_electrical
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# TODO: Make this a CMake Macro
add_custom_target(${PROJECT_NAME}_arduino ALL
    mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/arduino && arduino-cli core install arduino:avr && arduino-cli lib install "TimerOne" && arduino-cli lib install "TimerThree" && arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 ${CMAKE_CURRENT_SOURCE_DIR}/src/arduino --output-dir ${CMAKE_CURRENT_BINARY_DIR}/arduino
  BYPRODUCTS
    ${CMAKE_CURRENT_BINARY_DIR}/arduino/arduino.ino.eep
    ${CMAKE_CURRENT_BINARY_DIR}/arduino/arduino.ino.elf
    ${CMAKE_CURRENT_BINARY_DIR}/arduino/arduino.ino.hex
    ${CMAKE_CURRENT_BINARY_DIR}/arduino/arduino.ino.with_bootloader.bin
    ${CMAKE_CURRENT_BINARY_DIR}/arduino/arduino.ino.with_bootloader.hex
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arduino/arduino.ino
  USES_TERMINAL
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_indyav_electrical.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
