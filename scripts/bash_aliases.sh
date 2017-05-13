#!/bin/bash

# This script is meant to be sourced when using the ros_alarms package. It
# defines a few useful aliases and adds bash autocompletion for them.


_alarm_complete() {
	local ALARM

	# Iterate over the comma diliniated list of known alarms
	for ALARM in $(rosparam get /known_alarms); do

		# Skip any entry that does not match the string to complete
		if [[ -z "$2" || ! -z "$(echo ${ALARM:0:${#2}} | grep $2)" ]]; then

			# Append the alarm name to the autocomplete list
			if [[ ! -z "$( echo ${ALARM: -1} | grep ',')" ]]; then
				COMPREPLY+=( "${ALARM:0:-1}" )
			else
				COMPREPLY+=( "$ALARM" )
			fi
		fi
	done
}


# Define the alarm aliases
alias alist="rosparam get /known_alarms"
alias araise="rosrun ros_alarms raise"
alias aclear="rosrun ros_alarms clear"
alias areport="rosrun ros_alarms report"

# Registers the autocompletion function to be invoked for ros_alarms aliases
complete -F _alarm_complete araise
complete -F _alarm_complete aclear
complete -F _alarm_complete areport
