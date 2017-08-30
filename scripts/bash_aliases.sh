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
alias nhold="rosrun ros_alarms raise station-hold"

# Visualization
alias nviz="rviz -d \$CATKIN_DIR/src/NaviGator/navigator.rviz"
