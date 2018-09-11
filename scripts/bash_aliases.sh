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

# Tasks
alias nmove="runtask Move"

# Wrench
_nwrench_complete()
{
	local WRENCH
	local WRENCHES
	WRENCHES=( rc autonomous keyboard emergency )
	for WRENCH in "${WRENCHES[@]}"; do
		# Skip any entry that does not match the string to complete
		if [[ -z "$2" || ! -z "$(echo ${WRENCH:0:${#2}} | grep $2)" ]]; then
				COMPREPLY+=( "$WRENCH" )
		fi
	done
}
nwrench()
{
    rosservice call /wrench/select "topic: '$1'"
}
complete -F _nwrench_complete nwrench

# Alarms
alias nhold="rosrun ros_alarms raise station-hold"

# Visualization
alias nviz="rviz -d \$CATKIN_DIR/src/NaviGator/navigator.rviz"

# Development
alias navfmt="python2.7 -m flake8 --ignore E731 --max-line-length=120 \$(rosrun mil_tools list_python_files \$CATKIN_DIR/src/NaviGator __init__.py deprecated/ gnc/navigator_path_planner/lqRRT .cfg simulation/vmrc)"
alias navtest="(cd \$CATKIN_DIR; rosrun mil_tools catkin_tests_directory.py src/NaviGator)"

# Work around for bagging to SSD and copying to HDD when possible
# Sync bags from the SSD to the HDD
alias syncbags="rsync -hru --progress $HOME/bags-fast/ $HOME/bags/"
# Bag to the SSD (so buffer doesn't overflow)
alias bagfast="BAG_DIR=$HOME/bags-fast bag"
# Clear the bag fast dir on the SSD (run after it is synced)
alias purgebagfast="rm -r $HOME/bags-fast/*"
complete -F _bagging_complete bagfast
