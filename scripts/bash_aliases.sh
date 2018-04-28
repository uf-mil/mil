#!/bin/bash

# This script will be sourced by the MIL runcom file if the install script
# cloned this repository during a run or if the install script was run after
# it was cloned manually. It contains general MIL commands that have been
# aliased to make them faster or easier to execute. Becoming familiar with
# these will likely increase productivity, so it is recommended to do so.


# Source common utilities
source $CATKIN_DIR/src/mil_common/scripts/two_line_bash.sh
source $CATKIN_DIR/src/mil_common/scripts/ros_connect.sh
source $CATKIN_DIR/src/mil_common/scripts/wsmux.sh
source $CATKIN_DIR/src/mil_common/scripts/bag.sh
source $CATKIN_DIR/src/mil_common/scripts/tasks_autocomplete.sh

# Source submodule aliases if the submodules have been pulled
if [[ -f $CATKIN_DIR/src/mil_common/ros_alarms/scripts/bash_aliases.sh ]]; then
	source $CATKIN_DIR/src/mil_common/ros_alarms/scripts/bash_aliases.sh
fi


# Debugging for ROS networking
ros_env() {
	echo "ROS_IP=$ROS_IP"
	echo "ROS_HOSTNAME=$ROS_HOSTNAME"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
}


# Directory navigation
alias ws="cd \$CATKIN_DIR"
alias mc="cd \$CATKIN_DIR/src/mil_common"

# ROS helpers
imageproc() # Example usage: imageproc /camera/seecam
{
    ROS_NAMESPACE="$1" rosrun image_proc image_proc
}

# Bash sourcing
alias srcbrc="source ~/.bashrc"

# Workspace specific and shared tmux sockets
alias tmux="tmux -L \$(echo \$CATKIN_DIR | rev | cut -d '/' -f1 | rev)"
alias shared-tmux="tmux -L mil_ws"

# Catkin workspace management
alias cm="catkin_make -C \$CATKIN_DIR -j8"

# Simulation
alias killgazebo="killall -9 gazebo && killall -9 gzserver && killall -9 gzclient"

# Tasks
alias runtask="rosrun mil_tasks task_client run"
alias canceltask="rosrun mil_tasks task_client cancel"
alias listtasks="rosrun mil_tasks task_client list"

# Development
alias mcfmt="python2.7 -m flake8 --ignore E731 --max-line-length=120 \$(rosrun mil_tools list_python_files \$CATKIN_DIR/src/mil_common __init__.py drivers/pointgrey_camera_driver drivers/velodyne drivers/mil_passive_sonar drivers/roboteq ros_alarms txros deprecated/)"
alias mctest="(cd \$CATKIN_DIR; rosrun mil_tools catkin_tests_directory.py src/mil_common -i ros_alarms velodyne_driver velodyne_pointcloud pointgrey_camera_driver)"
