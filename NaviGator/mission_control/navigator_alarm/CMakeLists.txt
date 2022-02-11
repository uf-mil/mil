cmake_minimum_required(VERSION 3.0.2)
project(navigator_alarm)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    mil_tools
    nav_msgs
    ros_alarms
)

catkin_python_setup()

catkin_package()

