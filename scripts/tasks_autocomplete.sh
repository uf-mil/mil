_task_complete() {
	local TASK

	# Iterate over the comma diliniated list of known alarms
	for TASK in $(rosparam get /available_tasks | grep -oh "[A-Za-z0-9_ ]*"); do
		# Skip any entry that does not match the string to complete
		if [[ -z "$2" || ! -z "$(echo ${TASK:0:${#2}} | grep $2)" ]]; then
				COMPREPLY+=( "$TASK" )
		fi
	done
}


# Registers the autocompletion function to be invoked for ros_connect
complete -F _task_complete runtask
