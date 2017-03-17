cmake_minimum_required(VERSION 2.8.3)
project(uf_common)
find_package(catkin REQUIRED COMPONENTS message_generation message_runtime geometry_msgs tf actionlib interactive_markers std_msgs actionlib_msgs cmake_modules)
catkin_python_setup()

add_action_files(
  FILES
    MoveTo.action
)


add_message_files(
  FILES
  PoseTwistStamped.msg
  PoseTwist.msg
  VelocityMeasurements.msg
  Float64Stamped.msg
  VelocityMeasurement.msg
  Acceleration.msg
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs geometry_msgs uf_common
)
catkin_package(
    DEPENDS Eigen
    CATKIN_DEPENDS message_generation message_runtime geometry_msgs tf actionlib interactive_markers std_msgs actionlib_msgs
    INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} include
    LIBRARIES
)

include_directories(${EIGEN_INCLUDE_DIRS} include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
install(PROGRAMS scripts/interactive_marker_moveto scripts/velocitymeasurements_to_vector3 scripts/param_saver DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
