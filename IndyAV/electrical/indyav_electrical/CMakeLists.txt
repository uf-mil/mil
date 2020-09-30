cmake_minimum_required(VERSION 3.0.2)
project(indyav_electrical)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES indyav_electrical
#  CATKIN_DEPENDS roscpp rospy std_msgs
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
add_custom_target(${PROJECT_NAME}_sensors ALL
    mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/sensors && arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors --output-dir ${CMAKE_CURRENT_BINARY_DIR}/sensors
  BYPRODUCTS
    ${CMAKE_CURRENT_BINARY_DIR}/sensors/sensors.ino.eep
    ${CMAKE_CURRENT_BINARY_DIR}/sensors/sensors.ino.elf
    ${CMAKE_CURRENT_BINARY_DIR}/sensors/sensors.ino.hex
    ${CMAKE_CURRENT_BINARY_DIR}/sensors/sensors.ino.with_bootloader.bin
    ${CMAKE_CURRENT_BINARY_DIR}/sensors/sensors.ino.with_bootloader.hex
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/sensors.ino
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
