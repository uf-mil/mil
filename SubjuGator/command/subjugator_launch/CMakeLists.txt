cmake_minimum_required(VERSION 3.0.2)
project(subjugator_launch)
find_package(catkin REQUIRED roslaunch)
roslaunch_add_file_check(launch)
catkin_package()
