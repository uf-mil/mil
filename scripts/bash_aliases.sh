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
