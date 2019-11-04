cmake_minimum_required(VERSION 2.8.3)
project(magnetic_dynamic_compensation)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs roscpp tf std_msgs message_runtime nodelet)

add_message_files(
  FILES
  FieldInfo.msg
)
generate_messages(
    DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_generation geometry_msgs roscpp tf std_msgs message_runtime nodelet
    INCLUDE_DIRS
    LIBRARIES
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(magnetic_dynamic_compensation_nodelet src/nodelet.cpp)
target_link_libraries(magnetic_dynamic_compensation_nodelet ${catkin_LIBRARIES})
add_dependencies(magnetic_dynamic_compensation_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(magnetic_dynamic_compensation_nodelet ${PROJECT_NAME}_generate_messages_cpp)
