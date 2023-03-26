cmake_minimum_required(VERSION 3.0.2)
project(subjugator_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(catkin
    REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    rospy
    gazebo_ros
    roscpp
    message_generation xacro)

catkin_python_setup()


find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

add_library(
    subjugator_buoyancy
        src/subjugator_buoyancy.cc
)
set_target_properties(subjugator_buoyancy PROPERTIES COMPILE_FLAGS "-std=c++17 -Wall")
target_link_libraries(
    subjugator_buoyancy
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    subjugator_liftdrag
        src/subjugator_liftdrag.cc
)
set_target_properties(subjugator_liftdrag PROPERTIES COMPILE_FLAGS "-std=c++17 -Wall")
target_link_libraries(
    subjugator_liftdrag
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    subjugator_thrusters
        src/subjugator_thrusters.cc
        src/subjugator_thruster_config.cpp
)
set_target_properties(subjugator_thrusters PROPERTIES COMPILE_FLAGS "-std=c++17 -Wall")
target_link_libraries(
    subjugator_thrusters
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    subjugator_state_set
        src/subjugator_state_set.cpp
)
set_target_properties(subjugator_state_set PROPERTIES COMPILE_FLAGS "-std=c++17 -Wall")
target_link_libraries(
    subjugator_state_set
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)
add_dependencies(subjugator_state_set
        ${catkin_EXPORTED_TARGETS}
)

add_service_files(
  DIRECTORY srv
  FILES
  ResetGazebo.srv
  RunJob.srv
  SetTurbulence.srv
)

generate_messages(DEPENDENCIES std_msgs)

add_dependencies(subjugator_thrusters
    subjugator_msgs_generate_messages_cpp
    subjugator_msgs_generate_messages_py
    ${catkin_EXPORTED_TARGETS}
)

add_library(
    subjugator_test
        src/subjugator_test.cc
)
set_target_properties(subjugator_test PROPERTIES COMPILE_FLAGS "-std=c++17 -Wall")
target_link_libraries(
    subjugator_test
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES subjugator_buoyancy subjugator_thrusters subjugator_state_set subjugator_liftdrag
    CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime
    DEPENDS
)

# Generate Sub8's URDF
xacro_add_files(
  urdf/sub8.urdf.xacro
  urdf/sub9.urdf.xacro
  urdf/sub8_no_cams.urdf.xacro
  INSTALL DESTINATION urdf
  TARGET xacro_urdf
)
