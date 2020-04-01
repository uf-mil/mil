cmake_minimum_required(VERSION 2.8.3)
project(indyav_controller)
catkin_package()
find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  roscpp
)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(
  PIDController
    src/PIDController.cpp
)
set_target_properties(PIDController PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
target_link_libraries(
  PIDController
    ${catkin_LIBRARIES}
)

add_executable(velocity_control_node nodes/velocity_control_node.cpp)
target_link_libraries(velocity_control_node ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${catkin_LIBRARIES} PIDController)

add_dependencies (velocity_control_node ${catkin_EXPORTED_TARGETS})


