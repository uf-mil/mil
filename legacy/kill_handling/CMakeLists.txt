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
)

generate_messages(
    DEPENDENCIES kill_handling std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      message_generation
      message_runtime
      std_msgs
    INCLUDE_DIRS
      include
    LIBRARIES
      kill_handling
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/kill_on_cond scripts/kill scripts/kill_master scripts/clear DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(kill_handling STATIC include/kill_handling/broadcaster.h include/kill_handling/listener.h)
set_target_properties(kill_handling PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS kill_handling
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/kill_handling/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
