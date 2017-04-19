cmake_minimum_required(VERSION 2.8.3)
project(c3_trajectory_generator)

find_package(catkin
  REQUIRED COMPONENTS
    nav_msgs
    actionlib
    message_runtime
    message_generation
    mil_msgs
    tf
    cmake_modules
    ros_alarms
    mil_tools
)

add_service_files(
  FILES
    SetDisabled.srv
)

find_package(Eigen3 REQUIRED)

generate_messages(
    DEPENDENCIES
)

catkin_package(
  DEPENDS
    mil_msgs
  CATKIN_DEPENDS
    nav_msgs
    actionlib
    message_runtime
    message_generation
    mil_msgs
    tf
    ros_alarms
    mil_tools
  INCLUDE_DIRS
    include
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
)

include_directories(
    include
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_executable(c3_trajectory_generator
  src/C3Trajectory.cpp
  src/node.cpp
  src/AttitudeHelpers.cpp
)
target_link_libraries(c3_trajectory_generator ${catkin_LIBRARIES})
add_dependencies(c3_trajectory_generator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
set_target_properties(c3_trajectory_generator PROPERTIES COMPILE_FLAGS "-std=c++11 -O3")
