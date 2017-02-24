#!/bin/bash

# This script will be sourced by the MIL runcom file if the install script
# cloned this repository during a run or if the install script was run after
# it was cloned manually. It contains NaviGator specific commands that have
# been aliased to make them faster or easier to execute. Becoming familiar with
# these will likely increase productivity, so it is recommended to do so.

# Directory navigation
alias nav="cd $CATKIN_DIR/src/NaviGator"

# Networking
alias rnav="ros_connect -n ${HOSTNAMES[1]}"
alias sshnav="ssh navigator@${HOSTNAMES[1]} -Y"

# Missions
alias navm="rosrun navigator_missions run_mission"
alias navc="rosrun navigator_missions move_command"

# Alarms
alias naraise="rosrun navigator_alarm raise"
alias naclear="rosrun navigator_alarm clear"
alias navst="rosrun navigator_alarm raise station_hold"

# Rosbag Templates
export NAV_CORE_TOPICS="/odom /absodom /clock"
export NAV_STEREO_LEFT_TOPICS="/stereo/left/camera_info /stereo/left/image_raw"
export NAV_STEREO_RIGHT_TOPICS="/stereo/right/camera_info /stereo/right/image_raw"
export NAV_RIGHT_CAM_TOPICS="/right/right/image_raw /right/right/camera_info"
export NAV_MOTOR_TOPICS="/BL_motor/cmd /BL_motor/feedback /BL_motor/status /BR_motor/cmd /BR_motor/feedback /BR_motor/status /FL_motor/cmd /FL_motor/feedback /FL_motor/status /FR_motor/cmd /FR_motor/feedback /FR_motor/status"
export NAV_VELODYNE_TOPIC="/velodyne_points"
export NAV_SICK_TOPIC="/scan"
export NAV_STEREO_TOPICS="$NAV_STEREO_LEFT_TOPICS $NAV_STEREO_RIGHT_TOPICS"
export NAV_LQRRT_TOPICS="/ogrid_master /unclassified_markers /lqrrt/effor /lqrrt/focus /lqrrt/goal /lqrrt/impact /lqrrt/path /lqrrt/ref /lqrrt/tree /move_to/feedback /move_to/goal /move_to/result"
alias nav-bag-custom='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record $NAV_CORE_TOPICS '
alias nav-bag-stereo='bag-custom $NAV_STEREO_TOPICS'
alias nav-bag-front='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /stereo/right/camera_info /stereo/right/image_raw /odom /absodom /clock'
alias nav-bag-right='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /right/right/camera_info /right/right/image_raw /odom /absodom /clock'
alias nav-bag-down='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /down/left/camera_info /down/left/image_raw /odom /absodom /clock'
alias nav-bag-velodyne='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /velodyne_points /odom /absodom /clock'
alias nav-bag-sick='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /scan /odom /absodom /clock'
alias nav-bag-velodyne-stereo='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /velodyne_points /stereo/left/camera_info /stereo/left/image_raw /stereo/right/camera_info /stereo/right/image_raw /odom /absodom /clock'
alias nav-bag-sick-stereo='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /scan /stereo/left/camera_info /stereo/left/image_raw /stereo/right/camera_info /stereo/right/image_raw /odom /absodom /clock'
alias nav-bag-velodyne-right='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /velodyne_points /right/right/camera_info /right/right/image_raw /odom /absodom /clock'
alias nav-bag-sick-right='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /scan /right/right/camera_info /right/right/image_raw /odom /absodom /clock'
alias nav-bag-motors='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /c3_trajectory_generator/waypoint /trajectory /wrench/cmd /BL_motor/cmd /BL_motor/feedback /BL_motor/status /BR_motor/cmd /BR_motor/feedback /BR_motor/status /FL_motor/cmd /FL_motor/feedback /FL_motor/status /FR_motor/cmd /FR_motor/feedback /FR_motor/status /odom /absodom /clock'
alias nav-bag-status='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /c3_trajectory_generator/waypoint /trajectory /wrench/cmd /battery_monitor /adaptation /diagnostics /learn /alarm /alarm_raise /odom /absodom /clock'
alias nav-bag-hydrophones='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /hydrophones/debug /hydrophones/ping /hydrophones/pose /hydrophones/processed /velodyne_points /odom /absodom /clock'
alias nav-bag-hydrophones-stereo='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /stereo/left/camera_info /stereo/left/image_raw /stereo/right/camera_info /stereo/right/image_raw /hydrophones/debug /hydrophones/ping /hydrophones/pose /hydrophones/processed /velodyne_points /odom /absodom /clock'
alias nav-bag-lqrrt='mkdir -p ~/bags/"`date +%Y-%m-%d`" && cd ~/bags/"`date +%Y-%m-%d`" && rosbag record /stereo/left/camera_info /ogrid /unclassified_markers /stereo/left/image_raw /velodyne_points /odom /absodom /lqrrt/effor /lqrrt/focus /lqrrt/goal /lqrrt/impact /lqrrt/path /lqrrt/ref /lqrrt/tree /move_to/feedback /move_to/goal /move_to/result /clock'
