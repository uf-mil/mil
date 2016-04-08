cmake_minimum_required(VERSION 2.8.3)
project(sub8_actuator_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
# catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetValve.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xmega_connector
  CATKIN_DEPENDS message_runtime rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
