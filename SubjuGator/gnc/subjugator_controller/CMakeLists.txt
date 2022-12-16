cmake_minimum_required(VERSION 2.8.3)
project(subjugator_controller)
find_package(catkin REQUIRED dynamic_reconfigure)
catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/AdaptiveController.cfg
)
catkin_package()
