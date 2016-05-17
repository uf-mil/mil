cmake_minimum_required(VERSION 2.8.3)
project(sub8_gazebo)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin
    REQUIRED COMPONENTS
    std_msgs
    rospy
    gazebo_ros
    roscpp
    message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

add_library(
    sub8_buoyancy
        src/sub8_buoyancy.cc
)

set_target_properties(sub8_buoyancy PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    sub8_buoyancy
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    sub8_thrusters
        src/sub8_thrusters.cc
)

set_target_properties(sub8_thrusters PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    sub8_thrusters
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_service_files(
  DIRECTORY srv
  FILES
  ResetGazebo.srv
  RunJob.srv
  SetTurbulance.srv
)

generate_messages(DEPENDENCIES std_msgs)

add_dependencies(sub8_thrusters
    sub8_msgs_generate_messages_cpp
    sub8_msgs_generate_messages_py
    ${catkin_EXPORTED_TARGETS}
)

add_library(
    sub8_test
        src/sub8_test.cc
)

set_target_properties(sub8_test PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    sub8_test
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES duck_gazebo
    CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime
    DEPENDS system_lib
)
