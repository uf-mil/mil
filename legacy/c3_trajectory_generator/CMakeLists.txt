cmake_minimum_required(VERSION 2.8.3)
project(c3_trajectory_generator)
find_package(
  catkin
  REQUIRED COMPONENTS
    nav_msgs
    actionlib
    message_runtime
    message_generation
    kill_handling
    uf_common
    tf
    cmake_modules
)

add_service_files(
  FILES
  SetDisabled.srv
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
generate_messages(
    DEPENDENCIES
)

catkin_package(
  DEPENDS
    uf_common
  CATKIN_DEPENDS
    nav_msgs
    actionlib
    message_runtime
    message_generation
    uf_common
    tf
    kill_handling
  INCLUDE_DIRS
    include
  LIBRARIES
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(c3_trajectory_generator src/C3Trajectory.cpp src/node.cpp src/AttitudeHelpers.cpp)
target_link_libraries(c3_trajectory_generator ${catkin_LIBRARIES})
add_dependencies(c3_trajectory_generator ${catkin_EXPORTED_TARGETS})
add_dependencies(c3_trajectory_generator ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(
  ${PROJECT_NAME}
  uf_common_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
set_target_properties(c3_trajectory_generator PROPERTIES COMPILE_FLAGS -std=c++11)
