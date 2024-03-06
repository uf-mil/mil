#!/bin/bash
export MIL_CONFIG_DIR=$HOME/.mil
mkdir -p "$MIL_CONFIG_DIR"
export MIL_WS="$HOME/catkin_ws"
MIL_REPO="$MIL_WS/src/mil"

# Source ROS and local catkin
# finds command that initiated the current process
if [[ $(ps -p $$ | tail -n 1 | awk '{ print $4 }') == "zsh" ]]; then
	# zsh detected, sourcing appropriate setup scripts"
	source /opt/ros/noetic/setup.zsh
	source "$MIL_WS/devel/setup.zsh"
else
	# bash detected, sourcing appropriate setup scripts"
	source /opt/ros/noetic/setup.bash
	source "$MIL_WS/devel/setup.bash"
fi

# Script tools
# Helper for autocompleting given a list of choices for the first argument
_list_complete() {
	local THING
	THINGS=("$1")
	for THING in "${THINGS[@]}"; do
		# Skip any entry that does not match the string to complete
		if [[ -z $2 || -n "$(echo "${THING:0:${#2}}" | grep "$2")" ]]; then
			COMPREPLY+=("$THING")
		fi
	done
}

# Source anything in scripts/rc.d. This allows
# us to organize out aliases/rc so its not all in this script
for f in "$MIL_REPO"/scripts/rc.d/*; do
	. "$f"
done

# Source other aliases / tools
source "$MIL_WS/src/mil/SubjuGator/scripts/bash_aliases.sh"
source "$MIL_WS/src/mil/mil_common/ros_alarms/scripts/bash_aliases.sh"
source "$MIL_WS/src/mil/mil_common/scripts/bag.sh"
source "$MIL_WS/src/mil/mil_common/mil_missions/setup.bash"
source "$MIL_WS/src/mil/mil_common/perception/point_cloud_object_detection_and_recognition/setup.bash"
source "$MIL_WS/src/mil/NaviGator/scripts/bash_aliases.sh"

# Repo aliases
alias mil='cd $MIL_REPO'
alias cm='catkin_make -C $MIL_WS'
alias vrx='cd $MIL_REPO/NaviGator/simulation/VRX/vrx'

# General ROS aliases
ros_env() {
	echo "ROS_IP=$ROS_IP"
	echo "ROS_HOSTNAME=$ROS_HOSTNAME"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
}

# Camera helpers
imageproc() { # Example usage: imageproc /camera/seecam
	ROS_NAMESPACE="$1" rosrun image_proc image_proc
}
calibratecamera() {
	rosrun camera_calibration cameracalibrator.py --no-service-check --pattern=chessboard --square=0.063 --size=8x6 --disable_calib_cb_fast_check camera:="$1" image:="$1"/image_raw
}

# Bash sourcing
alias srcbrc="source ~/.bashrc"

# file searching alias
alias search_root='sudo find / \
	            -not \( -path /run -prune \) \
		        -not \( -path /proc -prune \) \
		        -not \( -path /boot -prune \) \
		        -not \( -path /cdrom -prune \) \
		        -not \( -path /lost+found -prune \) \
		        -not \( -path /root -prune \) \
		        -not \( -path /sbin -prune \) \
		        -not \( -path /snap -prune \) \
		        -not \( -path /sys -prune \) \
		        -not \( -path /tmp -prune \) \
		        -not \( -path /var -prune \) \
		        -not \( -path /vmlinuz -prune \) \
		        -not \( -path /vmlinuz.old -prune \) \
		        -print | grep -i'

alias search='find . -print | grep -i'
alias fd="fdfind"

# Gazebo aliases
alias gazebogui="rosrun gazebo_ros gzclient __name:=gzclient"

# Process killing aliases
alias killgazebo="killall -9 gzserver && killall -9 gzclient"
alias killros='$MIL_REPO/scripts/kill_ros.sh'
alias killprocess='$MIL_REPO/scripts/kill_process.sh'

startxbox() {
	rosservice call /wrench/select "topic: '/wrench/rc'"
	roslaunch navigator_launch shore.launch
}

# catkin_make for one specific package only
RED='\033[0;31m'
cmonly() {
	cd ~/catkin_ws || exit
	catkin_make --only-pkg-with-deps $1
	cd - >/dev/null || exit
	echo "${RED}!! Warning: Future calls to catkin_make will just build the '$1' package. To revert this, ensure you run 'cm' or 'cd ~/catkin_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\"' when you want to recompile the entire repository."
}

alias xbox=startxbox

# PYTHONPATH modifications
export PYTHONPATH="${HOME}/catkin_ws/src/mil/mil_common/axros/axros/src:${PYTHONPATH}"
