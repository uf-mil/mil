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
	local WORKING_DIRECTORY=$PWD
	local SELECTION
	local FILE

	# Get the list of catkin workspaces in the user's home directory
	_catkin_ws_complete

	# Handles command line arguments
	while (( $# > 0 )); do
		case $1 in
			-b|--bind)

				# Ensure that a client has connected via SSH
				if [[ -z "$SSH_CLIENT" ]]; then
					echo "This is not an SSH session, so there is no client to bind"
					SELECTION="false"
					shift $#

				# If a workspace name was entered, select that workspace
				elif [[ -f ~/$2/devel/setup.sh ]]; then
					SELECTION=$2
					shift 2

				# If no workspace name was entered, use the currently selected workspace
				elif [[ -z "$2" && -f ~/$(echo $CATKIN_DIR | rev | cut -d '/' -f1 | rev)/devel/setup.sh ]]; then
					SELECTION=$(echo $CATKIN_DIR | rev | cut -d '/' -f1 | rev)
					shift 1

				# Prompt the user to check themself if the selected workspace is not valid
				else
					echo "The specified workspace is not valid. I don't trust like that..."
					echo "Please specify the name of a workspace in the home directory"
					SELECTION="false"
					shift $#
				fi


				if [[ "$SELECTION" != "false" ]]; then
					source $WS_CONFIG_FILE

					# Replace any binding that already exists for this SSH client
					local REPLACED="false"
					for (( BINDING_INDEX=0; BINDING_INDEX < ${#WORKSPACE_BINDINGS[@]}; BINDING_INDEX++ )); do
						if [[ "$(echo ${WORKSPACE_BINDINGS[BINDING_INDEX]} | cut -d ':' -f1)" == "$(echo $SSH_CLIENT | cut -d ' ' -f1)" ]]; then
							WORKSPACE_BINDINGS[BINDING_INDEX]="$(echo $SSH_CLIENT | cut -d ' ' -f1):$SELECTION"
							REPLACED="true"
						fi
					done

					# Add a new binding for the client if no previous one was found
					if [[ "$REPLACED" != "true" ]]; then
						WORKSPACE_BINDINGS+=( "$(echo $SSH_CLIENT | cut -d ' ' -f1):$SELECTION" )
					fi

					# Rewrite the configuration file with the new binding
					echo "WORKSPACE_BINDINGS=(	${WORKSPACE_BINDINGS[0]}" > $WS_CONFIG_FILE
					for (( BINDING_INDEX=1; BINDING_INDEX < ${#WORKSPACE_BINDINGS[@]}; BINDING_INDEX++ )); do
						echo "			${WORKSPACE_BINDINGS[$BINDING_INDEX]}" >> $WS_CONFIG_FILE
					done
					echo ")" >> $WS_CONFIG_FILE

					echo "SSH connections from $(echo $SSH_CLIENT | cut -d ' ' -f1) are bound to $SELECTION"
				fi
				SELECTION="false"
				;;
			-c|--connect)
				SELECTION="false"
				source $WS_CONFIG_FILE

				# Iterate through the list of bindings
				for BINDING in ${WORKSPACE_BINDINGS[@]}; do
					if [[ "$(echo $BINDING | cut -d ':' -f1)" == "$(echo $SSH_CLIENT | cut -d ' ' -f1)" ]]; then

						# If the binding was found for this SSH client, find it in the home directory
						if [[ -f ~/$(echo $BINDING | cut -d ':' -f2)/devel/setup.sh ]]; then
							SELECTION=$(echo $BINDING | cut -d ':' -f2)

							# Source the workspace runcom file if it exists
							if [[ -f ~/$SELECTION/.wsrc ]]; then
								source ~/$SELECTION/.wsrc
							fi

						# Otherwise, prompt the user to check themself
						else
							echo "The bound workspace is not valid. I don't trust like that..."
							echo "Please bind to a workspace in the home directory"
							SELECTION="false"
							shift $#
						fi
					fi
				done
				shift 1
				;;
			-h|--help)
				echo "Usage: wsmux [OPTION]... [WORKSPACE]"
				echo "Quick catkin workspace switcher."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-b [WORKSPACE]	--bind			Bind an SSH client to a workspace"
				echo "-c		--connect		Select the SSH client's workspace"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List the detected workspaces"
				echo "-p [WORKSPACE]	--pull			Pull the latest changes from upstream"
				echo "-s		--show			Show the selected workspace"
				echo "-u		--unbind		Remove the binding for an SSH client"
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
			-p|--pull)
				SELECTION=""

				# Make sure the user understands what they are doing
				echo -n "Are you sure you want to delete changes on repository master branches? [y/N] "
				read SELECTION
				if [[ "$SELECTION" != "Y" && "$SELECTION" != "y" ]]; then
					SELECTION="false"
					shift $#

				# If a workspace name was entered, select that workspace
				elif [[ -f ~/$2/devel/setup.sh ]]; then
					SELECTION=$2
					shift 2

				# If no workspace name was entered, use the currently selected workspace
				elif [[ -z "$2" && \
					-f ~/$(echo $CATKIN_DIR | rev | cut -d '/' -f1 | rev)/devel/setup.sh ]]; then
					SELECTION=$(echo $CATKIN_DIR | rev | cut -d '/' -f1 | rev)
					shift 1

				# Prompt the user to check themself if the selected workspace is not valid
				else
					echo "The specified workspace is not valid. I don't trust like that..."
					echo "Please specify the name of a workspace in the home directory"
					SELECTION="false"
					shift $#
				fi

				if [[ "$SELECTION" != "false" ]]; then
					for FILE in ~/$SELECTION/src/*; do

						# Verify that the directory is a Git repository
						if [[ -d $FILE/.git ]]; then
							cd $FILE

							# Verify that the repository's working directory is clean
							if $(git diff-index --quiet HEAD --); then

								# Fetch the most recent changes from upstream
								git fetch upstream

								# Force the creation of a master branch from upstream
								git checkout -B master upstream/master

								# Checkout submodules to the commits specified
								git submodule update --init --recursive
							else
								echo "The repository at $FILE is dirty"
							fi
						fi
					done
					cd $WORKING_DIRECTORY
				fi
				SELECTION="false"
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
			-u|--unbind)

				# Ensure that a client has connected via SSH
				if [[ -z "$SSH_CLIENT" ]]; then
					echo "This is not an SSH session, so there is no client to unbind"
					SELECTION="false"
					shift $#

				elif [[ "$SELECTION" != "false" ]]; then
					source $WS_CONFIG_FILE

					# Remove any binding that exists for this SSH client
					local NEW_BINDINGS
					for (( BINDING_INDEX=0; BINDING_INDEX < ${#WORKSPACE_BINDINGS[@]}; BINDING_INDEX++ )); do
						if [[ "$(echo ${WORKSPACE_BINDINGS[BINDING_INDEX]} | cut -d ':' -f1)" != "$(echo $SSH_CLIENT | cut -d ' ' -f1)" ]]; then
							NEW_BINDINGS+=( "${WORKSPACE_BINDINGS[BINDING_INDEX]}" )
						fi
					done

					if [[ "${NEW_BINDINGS[@]}" != "${WORKSPACE_BINDINGS[@]}" ]]; then

						# Rewrite the configuration file with the new binding
						echo "WORKSPACE_BINDINGS=(	${NEW_BINDINGS[0]}" > $WS_CONFIG_FILE
						for (( BINDING_INDEX=1; BINDING_INDEX < ${#NEW_BINDINGS[@]}; BINDING_INDEX++ )); do
							echo "			${NEW_BINDINGS[$BINDING_INDEX]}" >> $WS_CONFIG_FILE
						done
						echo ")" >> $WS_CONFIG_FILE

						echo "SSH connections from $(echo $SSH_CLIENT | cut -d ' ' -f1) are no longer bound"
					else
						echo "SSH connections from $(echo $SSH_CLIENT | cut -d ' ' -f1) were not bound"
					fi
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
						SELECTION=$1

					# Otherwise, prompt the user to check themself
					else
						echo "The specified workspace is not valid. I don't trust like that..."
						echo "Please specify the name of a workspace in the home directory"
						SELECTION="false"
						shift $#
					fi
				fi
				shift 1
				;;
		esac
	done

	if [[ ! -z "$SELECTION" && "$SELECTION" != "false" ]]; then
			source ~/$SELECTION/devel/setup.sh
			export CATKIN_DIR=~/$SELECTION
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
