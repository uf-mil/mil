#!/bin/bash

# The bmux command is used to manage sourcing the bagging variables for
# different MIL vehicles. This prevents setting potentially hundreds of
# variables in each new shell. It also allows us to prefix the variables with
# bag_* as opposed to bag_{vehicle}_*. Each project with bagging variables
# should contain a scripts/bagging_variables.sh file. A bmux command to source
# the correct set of variables for a vehicle should be added to the bash runcom
# file of that vehicle's main user


BAGGING_VARIABLES_FILE="scripts/bagging_variables.sh"


_bagging_variables_file_complete() {
	local FILE

	# Check for bagging variables files in the catkin workspace repositories
	for FILE in $CATKIN_DIR/src/"$2"*; do

		# Skip any repository that does not contain a bagging variables file
		[[ -f $FILE/$BAGGING_VARIABLES_FILE ]] || continue

		# Append just the name of the repository to the autocomplete list
		COMPREPLY+=( `echo "$FILE" | rev | cut -d "/" -f1 | rev` )
	done
}

bmux() {
	local SELECTION
	local BAGGING_VARIABLES="`env | grep 'bag_' | cut -d '=' -f1 | cut -d ' ' -f2`"

	# Get the list of catkin workspaces containing bagging variables files
	_bagging_variables_file_complete

	# Handles command line arguments
	while [ "$#" -gt 0 ]; do
		case $1 in
			-c|--clear)
				if [ ! -z "$BAGGING_VARIABLES" ]; then
					for VARIABLE in $BAGGING_VARIABLES; do
						unset "$VARIABLE"
					done
				fi

				# Special variables used in the bag command
				unset BAG_ALWAYS
				unset BAG_HOME
				SELECTION=false
				shift 1
				;;
			-h|--help)
				echo "Usage: bmux [OPTION]... [REPOSITORY]"
				echo "Manager for sourcing bagging variables for different MIL vehicles."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-c		--clear			Clear current bagging variables"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List projects with bagging variables"
				echo "-s		--show			Show current bagging variables"
				SELECTION=false
				shift 1
				;;
			-l|--list)
				if [ ! -z "$COMPREPLY" ]; then
					echo "${COMPREPLY[@]}" | sed 's/ /  /g'
				fi
				SELECTION=false
				shift 1
				;;
			-s|--show)
				if [ ! -z "$BAGGING_VARIABLES" ]; then
					echo "$BAGGING_VARIABLES" | sed ':a;N;$!ba;s/\n/  /g'
				fi
				SELECTION=false
				shift 1
				;;
			-*)
				echo "Option $1 is not implemented."
				echo "Try 'bmux --help' for more information."
				SELECTION=false
				shift 1
				;;
			*)
				if [ "$SELECTION" != "false" ]; then
					if [ -f $CATKIN_DIR/src/"$1"/$BAGGING_VARIABLES_FILE ]; then
						SELECTION="$1"
					else
						echo "$CATKIN_DIR/src/$1 has no scripts/bagging_variables.sh file."
						echo "Try 'bmux --help' for more information."
						SELECTION=false
					fi
				fi
				shift 1
				;;
		esac
	done

	if [ ! -z "$SELECTION" ] && [ "$SELECTION" != "false" ]; then
			source $CATKIN_DIR/src/$SELECTION/$BAGGING_VARIABLES_FILE
	fi

	unset COMPREPLY
}


# Registers the autocompletion function to be invoked for bmux
complete -F _bagging_variables_file_complete bmux
