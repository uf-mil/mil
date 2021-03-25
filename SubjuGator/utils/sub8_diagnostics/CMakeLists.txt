cmake_minimum_required(VERSION 2.8.3)
project(sub8_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mil_tools
  roscpp
  nav_msgs
  geometry_msgs
  mil_msgs
)

# Any tools used to diagnose the core functionality of the sub should be put here
catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS
  mil_tools
  roscpp
 DEPENDS
  system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(parse_odom_node src/deprecated/parsing/parse_odom.cpp)
add_dependencies(parse_odom_node ${parse_odom_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parse_odom_node ${catkin_LIBRARIES})

add_executable(parse_wrench_node src/deprecated/parsing/parse_wrench.cpp)
add_dependencies(parse_wrench_node ${parse_wrench_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parse_wrench_node ${catkin_LIBRARIES})

add_executable(dynamic_move src/deprecated/dynamic_move.cpp)
add_dependencies(dynamic_move ${dynamic_move_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_move ${catkin_LIBRARIES})

add_executable(dynamic_yaw src/deprecated/dynamic_yaw.cpp)
add_dependencies(dynamic_yaw ${dynamic_yaw_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_yaw ${catkin_LIBRARIES})

add_executable(linear_acc src/backup/linear_acc.cpp)
add_dependencies(linear_acc ${linear_acc_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(linear_acc ${catkin_LIBRARIES})

add_executable(angular_acc src/backup/angular_acc.cpp)
add_dependencies(angular_acc ${angular_acc_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angular_acc ${catkin_LIBRARIES})

catkin_python_setup()
