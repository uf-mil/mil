cmake_minimum_required(VERSION 2.8.3)
project(mil_passive_sonar)
find_package(catkin REQUIRED COMPONENTS tf std_msgs message_runtime message_generation rospy geometry_msgs)
catkin_python_setup()

add_message_files(
  FILES
  ProcessedPing.msg
  Debug.msg
  Ping.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  FindPinger.srv
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS tf std_msgs message_runtime message_generation rospy
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/ping_printer scripts/ping_logger scripts/ping_plotter scripts/hydrophones DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
