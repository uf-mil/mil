cmake_minimum_required(VERSION 2.8.3)
project(sub8_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  eigen_conversions
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)

generate_dynamic_reconfigure_options(
  cfg/pd_controller.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sub8_msgs dynamic_reconfigure
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  include
)

add_executable(pd_controller nodes/pd_controller.cc)
target_link_libraries(pd_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(pd_controller PROPERTIES COMPILE_FLAGS "-std=c++11")


add_dependencies(pd_controller
    sub8_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

# add_executable(mpc nodes/mpc.cc)
# target_link_libraries(mpc ${catkin_LIBRARIES} ${Boost_LIBRARIES})
