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
alias vrx='cd $MIL_REPO/NaviGator/simulation/VRX/vrx'
alias cputemp='watch sensors'

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

# Preflight aliases
alias preflight='python3 $MIL_REPO/mil_common/utils/mil_tools/scripts/mil-preflight/main.py'
alias ntmux='$MIL_REPO/NaviGator/scripts/tmux_start.sh'

# Process killing aliases
alias killgazebo="killall -9 gzserver && killall -9 gzclient"
alias killros='$MIL_REPO/scripts/kill_ros.sh'
alias killprocess='$MIL_REPO/scripts/kill_process.sh'

xbox() {
	rosservice call /wrench/select "topic: '/wrench/rc'"
	roslaunch navigator_launch shore.launch device_input:="$1"
}

# catkin_make for one specific package only
RED='\033[0;31m'
# cm --> catkin_make
# cm --test --> catkin_make run_tests
# cm <package> --> catkin_make --only-pkg-with-deps <package>
# cm <package> --test --> catkin_make --only-pkg-with-deps <package> run_tests
cm() {
	if [ $# -eq 0 ]; then
		catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCATKIN_WHITELIST_PACKAGES="" -C "$MIL_WS"
		mv "$MIL_WS/build/compile_commands.json" "$MIL_WS"
	else
		if [[ $1 == "--test" ]]; then
			catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 run_tests -C "$MIL_WS"
			mv "$MIL_WS/build/compile_commands.json" "$MIL_WS"
		elif [[ $2 == "--test" ]]; then
			# Build specific package then run tests
			cd "$MIL_WS" || return
			catkin_make --only-pkg-with-deps "$1"
			catkin_make run_tests --only-pkg-with-deps "$1" -C "$MIL_WS"
			cd - >/dev/null || exit
			echo -e "${RED}!! Warning: Future calls to catkin_make will just build the '$1' package. To revert this, ensure you run 'cm' or 'cd $MIL_WS && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\"' when you want to recompile the entire repository.\e[0m"
		else
			# Build specific package
			cd "$MIL_WS" || return
			catkin_make --only-pkg-with-deps "$1"
			cd - >/dev/null || exit
			echo -e "${RED}!! Warning: Future calls to catkin_make will just build the '$1' package. To revert this, ensure you run 'cm' or 'cd $MIL_WS && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\"' when you want to recompile the entire repository.\e[0m"
		fi
	fi
}

# potentially borrowed from forrest
autopush() {
	git push origin +"${1:-HEAD}":refs/heads/autopush-cameron-"$(uuidgen --random | cut -c1-8)"-citmp
}

# uhhh maybe also borrowed from forrest
cw() {
	git add -u
	git commit -m "work"
}

dmb() {
	git diff "$(git merge-base --fork-point "$(git branch -l main master --format '%(refname:short)')" HEAD)"
}

subnet_ip() {
	ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){2}37\.[0-9]*' | grep -v '127.0.0.1'
}

rosdisconnect() {
	unset ROS_IP
	unset ROS_MASTER_URI
	echo "Disconnected! New values:"
	echo "ROS_IP=$ROS_IP"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
}

rosconnect() {
	# --no-subnet flag to avoid checking for the subnet
	# Usage: rosconnect --no-subnet <my_ip> <master_ip>
	if [[ $1 == "--no-subnet" ]]; then
		if [[ -z $2 || -z $3 ]]; then
			echo "Usage: rosconnect --no-subnet <my_ip> <master_ip>"
			return
		fi
		export ROS_IP=${2}
		export ROS_MASTER_URI="http://${3}:11311"
		echo "ROS_IP=$ROS_IP"
		echo "ROS_MASTER_URI=$ROS_MASTER_URI"
		return
	fi
	if [[ -n $(subnet_ip) ]]; then
		my_ip=$(subnet_ip)
		export ROS_IP=$my_ip
		export ROS_MASTER_URI="http://${1}:11311"
		echo "ROS_IP=$ROS_IP"
		echo "ROS_MASTER_URI=$ROS_MASTER_URI"
	else
		echo "No 37 subnet IP found, not setting ROS_IP or ROS_MASTER_URI"
	fi
}

rosnavconnect() {
	rosconnect "192.168.37.82"
}

rossubconnect() {
	rosconnect "192.168.37.60"
}

prettycp() {
	rsync --recursive --times --modify-window=2 --progress --verbose --itemize-changes --stats --human-readable "$1" "$2"
}

# Disambiguation function for ROS_MASTER_URI/GAZEBO_MASTER_URI
# sets the values to calculated expr 11340+$1 and 11350+$1 respectively
disambig() {
	if [ "$#" -ne 1 ]; then
		printf "Usage: disambig <plus_factor>\n\tSets the ROS_MASTER_URI and GAZEBO_MASTER_URI to 11340+<plus_factor> and 11350+<plus_factor> respectively to allow for parallel simulations"
		return
	fi
	export ROS_MASTER_URI="http://localhost:$((11340 + $1))"
	export GAZEBO_MASTER_URI="http://localhost:$((11350 + $1))"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
	echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
}

mount_ssd() {
	sudo mkdir -p /mnt/ssd
	sudo mount -t exfat /dev/sda1 /mnt/ssd
}

unmount_ssd() {
	sudo umount /mnt/ssd
}

# PYTHONPATH modifications
export PYTHONPATH="${HOME}/catkin_ws/src/mil/mil_common/perception/vision_stack/src:${HOME}/catkin_ws/src/mil/mil_common/axros/axros/src:${PYTHONPATH}"

# Ensure that VRX does _not_ exit on completion, as we're not in a time-limited
# environment, and exiting too early would not let us debug as much!
export VRX_EXIT_ON_COMPLETION=0
