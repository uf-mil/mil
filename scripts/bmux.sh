#!/bin/bash

# The bmux command is used to manage sourcing the bagging aliases for different
# MIL vehicles. This prevents setting potentially hundreds of aliases in each
# new shell. It also allows us to prefix the aliases with bag_* as opposed to
# bag_{vehicle}_*. Each project with bagging aliases should contain a
# scripts/bagging_aliases.sh file.


BAGGING_ALIAS_FILE="scripts/bagging_aliases.sh"


_bagging_aliases_complete() {
	local FILE

	# Check for bagging aliase files in the catkin workspace repositories
	for FILE in $CATKIN_DIR/src/*; do

		# Skip any repository that does not contain a bagging aliases file
		[[ -f $FILE/$BAGGING_ALIAS_FILE ]] || continue

		# Append just the name of the repository to the autocomplete list
		COMPREPLY+=( `echo "$FILE" | rev | cut -d "/" -f1 | rev` )
	done
}

bmux() {
	local SELECTION
	local BAGGING_ALIASES="`alias | grep 'bag' | cut -d '=' -f1 | cut -d ' ' -f2`"

	# Get the list of catkin workspaces containing bagging aliases files
	_bagging_aliases_complete

	# Handles command line arguments
	while [ "$#" -gt 0 ]; do
		case $1 in
			-h|--help)
				echo "Usage: bmux [OPTION] [REPOSITORY]..."
				echo "Manager for sourcing bagging aliases for different MIL vehicles."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List projects with bagging aliases"
				echo "-s		--show			Show current bagging aliases"
				echo "-c		--clear			Clear current bagging aliases"
				shift 1
				;;
			-l|--list)
				echo "$COMPREPLY" | sed ':a;N;$!ba;s/\n/  /g'
				shift 1
				;;
			-s|--show)
				if [ ! -z "$BAGGING_ALIASES" ]; then
					echo "$BAGGING_ALIASES" | sed ':a;N;$!ba;s/\n/  /g'
				fi
				shift 1
				;;
			-c|--clear)
				if [ ! -z "$BAGGING_ALIASES" ]; then
					for ALIAS in $BAGGING_ALIASES; do
						unalias "$ALIAS"
					done
				fi
				shift 1
				;;
			*)
				if [ ! -z "`echo $COMPREPLY | grep $1`" ]; then
					SELECTION="$1"
				else
					echo "Option $1 is not implemented."
					echo "Try 'bmux --help' for more information."
				fi
				shift 1
				;;
		esac
	done

	if [ ! -z "$SELECTION" ]; then
		if [ -f $CATKIN_DIR/src/$SELECTION/$BAGGING_ALIAS_FILE ]; then
			source $CATKIN_DIR/src/$SELECTION/$BAGGING_ALIAS_FILE
		fi
	fi

	unset COMPREPLY
}


# Registers the autocompletion function to be invoked for bmux
complete -F _bagging_aliases_complete bmux
