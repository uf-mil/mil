#!/bin/bash

# This script will be sourced by the bmux command and the variables will be
# used by the bag command. It is imperative that each variable is in the bag_*
# namespace and contains a space diliniated list of ROS topics. The only two
# exceptions to this rule are the BAG_ALWAYS and BAG_DIR variables.


# Define topics that should be in every bag
export BAG_ALWAYS="/odom /absodom /clock"

# Define the directory that the bags will be stored in
export BAG_DIR=~/bags


# Topic variables that can be used from the bag command
export bag_stereo_left_cam="/stereo/left/camera_info /stereo/left/image_raw"
export bag_stereo_right_cam="/stereo/right/camera_info /stereo/right/image_raw"
export bag_stereo_cam="$bag_stereo_left_cam $bag_stereo_right_cam"
export bag_right_cam="/right/right/image_raw /right/right/camera_info"
export bag_down_cam="/down/camera_info /down/image_raw"
export bag_velodyne="/velodyne_points"
export bag_sick="/scan"
export bag_motors="/BL_motor/cmd /BL_motor/feedback /BL_motor/status /BR_motor/cmd /BR_motor/feedback /BR_motor/status /FL_motor/cmd /FL_motor/feedback /FL_motor/status /FR_motor/cmd /FR_motor/feedback /FR_motor/status"
export bag_status="/c3_trajectory_generator/waypoint /trajectory /wrench/cmd /battery_monitor /adaptation /diagnostics /learn /alarm /alarm_raise"
export bag_hydrophones="/hydrophones/debug /hydrophones/ping /hydrophones/pose /hydrophones/processed "
export bag_lqrrt="/ogrid_master /unclassified_markers /lqrrt/effor /lqrrt/focus /lqrrt/goal /lqrrt/impact /lqrrt/path /lqrrt/ref /lqrrt/tree /move_to/feedback /move_to/goal /move_to/result"
