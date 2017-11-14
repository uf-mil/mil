cmake_minimum_required(VERSION 2.8.3)
project(navigator_tools)
find_package(catkin
  REQUIRED COMPONENTS
    rospy
    roslib
    std_msgs
    nav_msgs
    geometry_msgs
    tf
    dynamic_reconfigure
)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/BoundsConfig.cfg
)

catkin_package()
