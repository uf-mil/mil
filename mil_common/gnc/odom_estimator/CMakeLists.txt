cmake_minimum_required(VERSION 2.8.3)
project(odom_estimator)

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    pluginlib
    sensor_msgs
    eigen_conversions
    geometry_msgs
    tf
    nodelet
    tf_conversions
    std_msgs
    message_generation
    cmake_modules
    mil_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_message_files(
  FILES
    Info.msg
)
add_service_files(
  FILES
    SetIgnoreMagnetometer.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    pluginlib
    sensor_msgs
    eigen_conversions
    geometry_msgs
    tf
    mil_msgs
    nodelet
    tf_conversions
    std_msgs
    message_runtime
  INCLUDE_DIRS
    include
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
)

include_directories(
    include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
)

add_library(odom_estimator_nodelet src/nodelet.cpp)
target_link_libraries(odom_estimator_nodelet ${catkin_LIBRARIES})
add_dependencies(odom_estimator_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_estimator_nodelet ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(odom_estimator_nodelet PROPERTIES COMPILE_FLAGS "-O3 -std=c++0x")

add_executable(test_unscented_transform src/test_unscented_transform.cpp)
target_link_libraries(test_unscented_transform ${catkin_LIBRARIES})
add_dependencies(test_unscented_transform ${catkin_EXPORTED_TARGETS})
set_target_properties(test_unscented_transform PROPERTIES COMPILE_FLAGS  "-O3 -std=c++0x")

add_executable(test_mag src/test_mag.cpp)
target_link_libraries(test_mag ${catkin_LIBRARIES})
add_dependencies(test_mag ${catkin_EXPORTED_TARGETS})
set_target_properties(test_mag PROPERTIES COMPILE_FLAGS "-O3 -std=c++0x")

