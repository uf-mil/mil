#!/bin/bash
export MIL_CONFIG_DIR=$HOME/.mil
mkdir -p $MIL_CONFIG_DIR
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
MIL_REPO="$MIL_WS/src/mil"

# Source ROS and local catkin
source /opt/ros/melodic/setup.bash
source $MIL_WS/devel/setup.bash

# Script tools
# Helper for autocompleting given a list of choices for the first argument
_list_complete()
{
    local THING
    THINGS=($1)
    for THING in "${THINGS[@]}"; do
            # Skip any entry that does not match the string to complete
            if [[ -z "$2" || ! -z "$(echo ${THING:0:${#2}} | grep $2)" ]]; then
                            COMPREPLY+=( "$THING" )
            fi
    done
}

# Source anything in scripts/rc.d. This allows
# us to organize out aliases/rc so its not all in this script
for f in $MIL_REPO/scripts/rc.d/*
do
	. "$f"
done

# Source other aliases / tools
source $MIL_WS/src/mil/SubjuGator/scripts/bash_aliases.sh
source $MIL_WS/src/mil/mil_common/ros_alarms/scripts/bash_aliases.sh
source $MIL_WS/src/mil/mil_common/scripts/bag.sh
source $MIL_WS/src/mil/mil_common/mil_missions/setup.bash
source $MIL_WS/src/mil/mil_common/perception/point_cloud_object_detection_and_recognition/setup.bash
source $MIL_WS/src/mil/NaviGator/scripts/bash_aliases.sh
source $MIL_WS/src/mil/IndyAV/scripts/bash_aliases.sh

# Repo aliases
alias mil="cd $MIL_REPO"
alias cm="catkin_make -C $MIL_WS"
alias cm_arm="catkin_make -C $MIL_WS -DCATKIN_BLACKLIST_PACKAGES='sub8_gazebo;sub8_simulation;navigator_2dsim;navigator_gazebo;vrx;vrx_docker;indyav_gazebo;navigator_judgespanel;mil_gazebo;rviz_satellite'"

# General ROS aliases
ros_env() {
	echo "ROS_IP=$ROS_IP"
	echo "ROS_HOSTNAME=$ROS_HOSTNAME"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
}

# Camera helpers
imageproc() # Example usage: imageproc /camera/seecam
{
    ROS_NAMESPACE="$1" rosrun image_proc image_proc
}
calibratecamera()
{
  rosrun camera_calibration cameracalibrator.py --no-service-check --pattern=chessboard --square=0.063 --size=8x6 --disable_calib_cb_fast_check camera:=$1 image:=$1/image_raw
}

# Bash sourcing
alias srcbrc="source ~/.bashrc"

# Gazebo aliases
alias gazebogui="rosrun gazebo_ros gzclient __name:=gzclient"
# Simulation
alias killgazebo="killall -9 gzserver && killall -9 gzclient"
