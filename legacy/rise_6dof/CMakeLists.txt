cmake_minimum_required(VERSION 2.8.3)
project(rise_6dof)
find_package(catkin REQUIRED COMPONENTS nav_msgs dynamic_reconfigure message_runtime message_generation rospy uf_common kill_handling tf geometry_msgs)
catkin_python_setup()

add_service_files(
  FILES
  SendConstantWrench.srv
)

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
)

generate_messages(
    DEPENDENCIES geometry_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS nav_msgs dynamic_reconfigure message_runtime message_generation rospy uf_common kill_handling tf geometry_msgs
    INCLUDE_DIRS
    LIBRARIES
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/test_rviz scripts/joy_to_posetwist scripts/do_spiral scripts/test scripts/rise_6dof DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
