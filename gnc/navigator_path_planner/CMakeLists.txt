cmake_minimum_required(VERSION 2.8.3)
project(navigator_path_planner)
find_package(catkin
  REQUIRED
  COMPONENTS roscpp rospy std_msgs geometry_msgs nav_msgs actionlib actionlib_msgs
)
catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES Move.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs nav_msgs
)

