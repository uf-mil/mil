#!/bin/bash

# The wsmux command makes it easy to switch between catkin workspaces. This
# is mainly intended for use on vehicles as each user will have their own
# workspace containing repositories that reflect their branches. In the event
# that they need to test unmerged code in the field, they will run the node
# from their workspace. It can also be used to manage multiple workspaces on
# user machines, if you're into that sort of thing.


WS_MARKER=".catkin_workspace"


_catkin_ws_complete() {
	local FILE

	# Check for workspaces in the home directory based on the argument
	for FILE in ~/"$2"*; do

		# Skip any directory that does not contain the marker file
		if [[ -f $FILE/$WS_MARKER ]]; then

			# Append just the name of the workspace folder to the autocomplete list
			COMPREPLY+=( `echo "$FILE" | rev | cut -d "/" -f1 | rev` )
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
				echo "Usage: wsmux [OPTION]... [CATKIN_WORKSPACE]"
				echo "Quick catkin workspace switcher."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List the detected catkin workspaces"
				echo "-s		--show			Show the selected catkin workspace"
				SELECTION="false"
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
					echo "$ROS_PACKAGE_PATH" | cut -d ":" -f1 | sed "s@/src@@"
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

					# If the workspace was specified as a path, use it as is
					if [[ -f $1/devel/setup.sh ]]; then
						CATKIN_DIR=`pwd`/$1
						SELECTION=$CATKIN_DIR/devel/setup.sh

					# If a workspace name was passed, find it in the home directory
					elif [[ -f ~/$1/devel/setup.sh ]]; then
						CATKIN_DIR=~/$1
						SELECTION=$CATKIN_DIR/devel/setup.sh

					# If neither of these were the case, prompt the user to check themself
					else
						echo "The specified workspace is not valid. I don't trust like that..."
						echo "Please specify a path or the name of a workspace in the home directory"
						SELECTION="false"
					fi
				fi
				shift 1
				;;
		esac
	done

	if [[ ! -z "$SELECTION" && "$SELECTION" != "false" ]]; then
			source $SELECTION
			export CATKIN_DIR
	fi

	unset COMPREPLY
}


# Registers the autocompletion function to be invoked for wsmux
complete -F _catkin_ws_complete wsmux
