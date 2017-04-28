#!/bin/bash

# The wsmux command makes it easy to switch between catkin workspaces. This
# is mainly intended for use on vehicles as each user will have their own
# workspace containing repositories that reflect their branches. In the event
# that they need to test unmerged code in the field, they will run the node
# from their workspace. It can also be used to manage multiple workspaces on
# user machines, if you're into that sort of thing.


WS_CONFIG_FILE=$MIL_CONFIG_DIR/wsmux.conf

# This file is created in the root directory of a catkin workspace
WS_MARKER=".catkin_workspace"


_catkin_ws_complete() {
	local FILE

	# Check for workspaces in the home directory based on the argument
	for FILE in ~/"$2"*; do

		# Skip any directory that does not contain the marker file
		if [[ -f $FILE/$WS_MARKER ]]; then

			# Append just the name of the workspace folder to the autocomplete list
			COMPREPLY+=( $(echo "$FILE" | rev | cut -d '/' -f1 | rev) )
		fi
	done
}

wsmux() {
	local SELECTION

	# Get the list of catkin workspaces in the user's home directory
	_catkin_ws_complete

	# Handles command line arguments
	while (( $# > 0 )); do
		case $1 in
			-h|--help)
				echo "Usage: wsmux [OPTION]... [WORKSPACE]"
				echo "Quick catkin workspace switcher."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-b [WORKSPACE]	--bind			Bind an SSH client to a workspace"
				echo "-c		--connect		Select the SSH client's workspace"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List the detected workspaces"
				echo "-s		--show			Show the selected workspace"
				SELECTION="false"
				shift 1
				;;
			-b|--bind)

				# Ensure that a client has connected via SSH
				if [[ -z "$SSH_CLIENT" ]]; then
					echo "This is not an SSH session, so there is no client to bind"

				# Prompt the user to check themself if the workspace is not valid
				elif [[ ! -f ~/$2/devel/setup.sh ]]; then
					echo "The specified workspace is not valid. I don't trust like that..."
					echo "Please specify the name of a workspace in the home directory"

				else
					source $WS_CONFIG_FILE

					# Replace any binding that already exists for this SSH client
					local REPLACED="false"
					for (( BINDING_INDEX=0; BINDING_INDEX < ${#WORKSPACE_BINDINGS[@]}; BINDING_INDEX++ )); do
						if [[ "$(echo ${WORKSPACE_BINDINGS[BINDING_INDEX]} | cut -d ':' -f1)" == "$(echo $SSH_CLIENT | cut -d ' ' -f1)" ]]; then
							WORKSPACE_BINDINGS[BINDING_INDEX]="$(echo $SSH_CLIENT | cut -d ' ' -f1):$2"
							REPLACED="true"
						fi
					done

					# Add a new binding for the client if no previous one was found
					if [[ "$REPLACED" != "true" ]]; then
						WORKSPACE_BINDINGS+=( "$(echo $SSH_CLIENT | cut -d ' ' -f1):$2" )
					fi

					# Rewrite the configuration file with the new binding
					echo "WORKSPACE_BINDINGS=(	${WORKSPACE_BINDINGS[0]}" > $WS_CONFIG_FILE
					for (( BINDING_INDEX=1; BINDING_INDEX < ${#WORKSPACE_BINDINGS[@]}; BINDING_INDEX++ )); do
						echo "			${WORKSPACE_BINDINGS[$BINDING_INDEX]}" >> $WS_CONFIG_FILE
					done
					echo ")" >> $WS_CONFIG_FILE
				fi
				SELECTION="false"
				shift 1
				;;
			-c|--connect)
				SELECTION="false"
				source $WS_CONFIG_FILE

				# Iterate through the list of bindings
				for BINDING in ${WORKSPACE_BINDINGS[@]}; do
					if [[ "$(echo $BINDING | cut -d ':' -f1)" == "$(echo $SSH_CLIENT | cut -d ' ' -f1)" ]]; then

						# If the binding was found for this SSH client, find it in the home directory
						if [[ -f ~/$(echo $BINDING | cut -d ':' -f2)/devel/setup.sh ]]; then
							SELECTION=~/$(echo $BINDING | cut -d ':' -f2)

						# Otherwise, prompt the user to check themself
						else
							echo "The specified workspace is not valid. I don't trust like that..."
							echo "Please specify the name of a workspace in the home directory"
							SELECTION="false"
						fi
					fi
				done
				shift 1
				;;
			-l|--list)
				if [[ ! -z "COMPREPLY" ]]; then
					echo "${COMPREPLY[@]}" | sed 's/ /  /g'
				fi
				SELECTION="false"
				shift 1
				;;
			-s|--show)
				if [[ ! -z "$ROS_PACKAGE_PATH" ]]; then
					echo -n "Currently sourced catkin workspace: "
					echo "$ROS_PACKAGE_PATH" | cut -d ':' -f1 | sed "s@/src@@"
				else
					echo "No catkin workspace is currently sourced"
				fi
				SELECTION="false"
				shift 1
				;;
			-*)
				echo "Option $1 is not implemented."
				echo "Try 'wsmux --help' for more information."
				SELECTION="false"
				shift 1
				;;
			*)
				if [[ "$SELECTION" != "false" ]]; then

					# If a workspace name was passed, find it in the home directory
					if [[ -f ~/$1/devel/setup.sh ]]; then
						SELECTION=~/$1

					# Otherwise, prompt the user to check themself
					else
						echo "The specified workspace is not valid. I don't trust like that..."
						echo "Please specify the name of a workspace in the home directory"
						SELECTION="false"
					fi
				fi
				shift 1
				;;
		esac
	done

	if [[ ! -z "$SELECTION" && "$SELECTION" != "false" ]]; then
			source $SELECTION/devel/setup.sh
			export CATKIN_DIR=$SELECTION
	fi

	unset COMPREPLY
}

# Generates the configuration file if it does not exist
if [[ ! -f $WS_CONFIG_FILE ]]; then
	echo "WORKSPACE_BINDINGS=(	" > $WS_CONFIG_FILE
	echo ")" >> $WS_CONFIG_FILE
fi

# Registers the autocompletion function to be invoked for wsmux
complete -F _catkin_ws_complete wsmux
