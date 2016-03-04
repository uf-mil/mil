cmake_minimum_required(VERSION 2.8.3)
project(kill_handling)
find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs)
catkin_python_setup()

add_message_files(
  FILES
  KillsStamped.msg
  Kill.msg
)

add_service_files(
  FILES
  SetKill.srv


generate_messages(
    DEPENDENCIES kill_handling std_msgs
)

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS message_generation message_runtime std_msgs
    INCLUDE_DIRS include# TODO include
    LIBRARIES # TODO
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/kill_on_cond scripts/kill scripts/kill_master scripts/clear DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
