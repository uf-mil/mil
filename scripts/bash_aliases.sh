#!/bin/bash

# This script will be sourced by the MIL runcom file if the install script
# cloned this repository during a run or if the install script was run after
# it was cloned manually. It contains SubjuGator specific commands that have
# been aliased to make them faster or easier to execute. Becoming familiar with
# these will likely increase productivity, so it is recommended to do so.


# Directory navigation
alias sub="cd \$CATKIN_DIR/src/SubjuGator"

# Networking
alias rsub="ros_connect -n ${HOSTNAMES[0]}"
alias sshsub="ssh sub8@${HOSTNAMES[0]} -Y"

# Missions
alias subm="rosrun sub8_missions run_mission"
alias subc="rosrun sub8_missions move_command"

# Cameras
alias subfps="rostopic hz $bag_front_cams $bag_down_cam"

# Visualization
alias subrviz="rviz -d \$CATKIN_DIR/src/SubjuGator/sub.rviz"

# Formatting (update with Jenkinsfile)
alias subfmt="python2.7 -m flake8 --ignore E731 --max-line-length=120 --exclude=__init__.py \$CATKIN_DIR/src/SubjuGator"
