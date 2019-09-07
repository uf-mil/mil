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

# Actuators
subvalveopen()
{
 rosservice call /set_valve "{actuator: $1, opened: true}"
}
subvalveclose()
{
 rosservice call /set_valve "{actuator: $1, opened: false}"
}
subvalveopenclose()
{
 rosservice call /set_valve "{actuator: $1, opened: true}"
 sleep $2
 rosservice call /set_valve "{actuator: $1, opened: false}"
}

# Missions
alias submove="runmission Move"

# Cameras
alias subfps="rostopic hz $bag_front_cams $bag_down_cam"

# Visualization
alias subviz="rviz -d \$CATKIN_DIR/src/SubjuGator/sub.rviz"

# Development
alias subfmt="python2.7 -m flake8 --ignore E731 --max-line-length=120 \$(rosrun mil_tools list_python_files \$CATKIN_DIR/src/SubjuGator __init__.py deprecated/)"
alias subtest="(cd \$CATKIN_DIR; rosrun mil_tools catkin_tests_directory.py src/SubjuGator)"


# Set robot model param, for using rviz when playing bags
alias submodel="roslaunch sub8_launch upload_urdf.launch"
