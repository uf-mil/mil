#!/bin/bash

# This script will be sourced by the MIL runcom file if the install script
# cloned this repository during a run or if the install script was run after
# it was cloned manually. It contains general MIL commands that have been
# aliased to make them faster or easier to execute. Becoming familiar with
# these will likely increase productivity, so it is recommended to do so.


# Source external alias modules
source $CATKIN_DIR/src/mil_common/scripts/two_line_bash.sh
source $CATKIN_DIR/src/mil_common/scripts/ros_connect.sh
source $CATKIN_DIR/src/mil_common/scripts/wsmux.sh
source $CATKIN_DIR/src/mil_common/scripts/bmux.sh


# Directory navigation
alias swc="cd $CATKIN_DIR/src/mil_common"

# Bash sourcing
alias srcbrc="source ~/.bashrc"

# Catkin workspace management
alias cm="catkin_make -C $CATKIN_DIR -j8"

# Alarms
alias araise="rosrun ros_alarms raise"
alias aclear="rosrun ros_alarms clear"
alias areport="rosrun ros_alarms report"

# Simulation
alias killgazebo="killall -9 gazebo && killall -9 gzserver && killall -9 gzclient"
