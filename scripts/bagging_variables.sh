#!/bin/bash

# This script will be sourced by the bmux command and the variables will be
# used by the bag command. It is imperative that each variable is in the bag_*
# namespace and contains a space diliniated list of ROS topics. The only two
# exceptions to this rule are the BAG_ALWAYS and BAG_DIR variables.


# Define topics that should be in every bag
export BAG_ALWAYS="/odom /absodom /clock /tf /tf_static"

# Define the directory that the bags will be stored in
export BAG_DIR=~/bags


# Topic variables that can be used from the bag command
export bag_front_left_cam="/camera/front/left/camera_info /camera/front/left/image_raw"
export bag_front_right_cam="/camera/front/right/camera_info /camera/front/right/image_raw"
export bag_front_cams="$bag_front_left_cam $bag_front_right_cam"
export bag_down_cam="/camera/down/left/camera_info /camera/down/left/image_raw"
export bag_blueview_ranges="/blueview_driver/ranges"
export bag_blueview_color="/blueview_driver/image_color"
export bag_blueview_mono="/bluewview_driver/image_mono"
export bag_blueview="$bag_blueview_ranges $bag_blueview_color $bag_blueview_mono"
