#!/bin/bash

# This script will be sourced by the bmux command and the variables will be
# used by the bag command. It is imperative that each variable is in the bag_*
# namespace and contains a space diliniated list of ROS topics. The only two
# exceptions to this rule are the BAG_ALWAYS and BAG_DIR variables.


# Define topics that should be in every bag
export BAG_ALWAYS="/odom /absodom /clock /tf /tf_static /diagnostics"

# Define the directory that the bags will be stored in
export BAG_DIR=~/bags


# Topic variables that can be used from the bag command
export bag_front_left_cam="/camera/front/left/camera_info /camera/front/left/image_raw"
export bag_front_right_cam="/camera/front/right/camera_info /camera/front/right/image_raw"
export bag_front_cam="$bag_front_left_cam $bag_front_right_cam"
export bag_starboard_cam="/camera/starboard/image_raw /camera/starboard/camera_info"
export bag_down_cam="/camera/down/camera_info /camera/down/image_raw"
export bag_velodyne="/velodyne_points"
export bag_sick="/scan"
export bag_motors="/BL_motor/cmd /BL_motor/feedback /BL_motor/status /BR_motor/cmd /BR_motor/feedback /BR_motor/status /FL_motor/cmd /FL_motor/feedback /FL_motor/status /FR_motor/cmd /FR_motor/feedback /FR_motor/status /battery_monitor"
export bag_hydrophones_ping="/hydrophones/ping"
export bag_hydrophones_direction="/hydrophones/direction /hydrophones/processed /hydrophones/debug"
export bag_hydrophones_multilateration="/hydrophones/position"
export bag_hydrophones="$bag_hydrophones_ping $bag_hydrophones_direction $bag_hydrophones_multilateration"
export bag_object_detection="/database/objects"
export bag_controller="/wrench/cmd /wrench/selected"
export bag_guidance="/ogrid_master /trajectory/selected /trajectory/cmd /lqrrt/path /lqrrt/tree /lqrrt/goal /lqrrt/focus /lqrrt/effort /lqrrt/sampspace /lqrrt/effort /move_to/feedback /move_to/goal /move_to/result"
export bag_gnc="$bag_guidance $bag_controller $bag_motors"
export bag_kill="$bag_front_cam $bag_velodyne $bag_gnc $bag_starboard_cam $bag_motors"
