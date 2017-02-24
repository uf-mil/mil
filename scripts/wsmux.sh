#!/bin/bash

# The wsmux command makes it easy to switch between catkin workspaces. This
# is mainly intended for use on vehicles as each user will have their own
# workspace containing repositories that reflect their branches. In the event
# that they need to test unmerged code in the field, they will run the node
# from their workspace. It can also be used to manage multiple workspaces on
# user machines, if you're into that sort of thing.


_catkin_ws_complete() {
	local FILE
	local WS_MARKER=".catkin_workspace"

	# Check for workspaces in the home directory based on the argument
	for FILE in ~/"$2"*; do

		# Skip any directory that does not contain the marker file
		[[ -f $FILE/$WS_MARKER ]] || continue

		# Append just the name of the workspace folder to the autocomplete list
		COMPREPLY+=( `echo "$FILE" | rev | cut -d "/" -f1 | rev` )
	done
}

wsmux() {
	if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then

		# Print the help menu if the user passes in -h or --help
		echo "Usage: wsmux CATKIN_WORKSPACE"
		echo "Quick catkin workspace switcher."
		echo ""
		echo "Option		GNU long option		Meaning"
		echo "-h		--help			Display the help menu"
	else

		# If the workspace was specified as a path, use it as is
		if [ -f "$1"/devel/setup.sh ]; then
			source "$1"/devel/setup.sh

		# If a workspace name was passed, find it in the home directory
		elif [ -f ~/"$1"/devel/setup.sh ]; then
			source ~/"$1"/devel/setup.sh

		# If neither of these were the case, informe the user that they were wrong
		else
			echo "The specified workspace is not valid. I don't trust like that..."
			echo "Please specify a path or the name of a workspace in the home directory"
		fi
	fi
}


# Registers the autocompletion function to be invoked for wsmux
complete -F _catkin_ws_complete wsmux
