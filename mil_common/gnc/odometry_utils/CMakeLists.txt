cmake_minimum_required(VERSION 2.8.3)
project(odometry_utils)

find_package(catkin
  REQUIRED COMPONENTS
    nav_msgs
    roscpp
    eigen_conversions
    odom_estimator
    geometry_msgs
    tf
    nodelet
    tf_conversions
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
    nav_msgs
    roscpp
    eigen_conversions
    odom_estimator
    geometry_msgs
    tf
    nodelet
    tf_conversions
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(transform_odometry src/transform_odometry.cpp)
target_link_libraries(transform_odometry ${catkin_LIBRARIES})
add_dependencies(transform_odometry ${catkin_EXPORTED_TARGETS})
set_target_properties(transform_odometry PROPERTIES COMPILE_FLAGS "-O3 -std=c++11")

add_library(odometry_to_tf src/odometry_to_tf.cpp)
target_link_libraries(odometry_to_tf ${catkin_LIBRARIES})
add_dependencies(odometry_to_tf ${catkin_EXPORTED_TARGETS})
set_target_properties(odometry_to_tf PROPERTIES COMPILE_FLAGS "-O3 -std=c++11")

