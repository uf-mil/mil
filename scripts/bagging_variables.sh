#!/bin/bash

# This script will be sourced by the bag command as a configuration file. It is
# imperative that each variable is in the bag_* namespace and contains a space
# deliniated list of ROS topics. The only two exceptions to this rule are the
# BAG_ALWAYS and BAG_DIR variables.


# Define topics that should be in every bag
export BAG_ALWAYS="/odom /absodom /clock /tf /tf_static /dvl/range"

# Define the directory that the bags will be stored in
export BAG_DIR=~/bags


# Define the type of image to use for camera topics
export CAM_SUFFIX="image_rect_color"


# Topic variables that can be used from the bag command
export bag_front_left_cam="/camera/front/left/camera_info /camera/front/left/$CAM_SUFFIX"
export bag_front_right_cam="/camera/front/right/camera_info /camera/front/right/$CAM_SUFFIX"
export bag_front_cams="$bag_front_left_cam $bag_front_right_cam"
export bag_down_cam="/camera/down/left/camera_info /camera/down/left/$CAM_SUFFIX"
export bag_blueview_ranges="/blueview_driver/ranges"
export bag_blueview_color="/blueview_driver/image_color"
export bag_blueview_mono="/blueview_driver/image_mono"
export bag_blueview="$bag_blueview_ranges $bag_blueview_color $bag_blueview_mono"
export bag_controller="/pd_output /rise_6dof/parameter_descriptions /rise_6dof/parameter_updates"
export bag_navigation="/wrench /wrench_actual /wrench_error /c3_trajectory_generator/trajectory_v /c3_trajectory_generator/waypoint /trajectory"
export bag_thrusters="/thrusters/thrust /thrusters/status/FLV /thrusters/status/FLH /thrusters/status/FRV /thrusters/status/FRH /thrusters/status/BLV /thrusters/status/BLH /thrusters/status/BRV /thrusters/status/BRH"
export bag_gnc="$bag_controller $bag_navigation $bag_thrusters"
export bag_kill="$bag_gnc $bag_front_left_cam $bag_blueview"
