#!/bin/bash

# The bag command works in tandum with the bmux command. Once the bagging
# variables have been loaded with bmux, the bag command can be used with
# autocompletion to select topics. It is mainly a convenience layer for
# rosbag; however, it does introduce a few features. A list of BAG_ALWAYS
# topics can include topics that every bag should contain, bag directory
# management is handled cleanly, and naming bags is... strongly encouraged.


VARIABLE_PREFIX="bag_"


_bagging_complete() {
	local VARIABLE

	# Append all ROS topics to the autocomplete list if the string to autocomplete begins with a '/' character
	if [[ "${2:0:1}" == '/' ]]; then
		COMPREPLY+=( $(rostopic list 2> /dev/null) )

	else

		# Otherwise, iterate over all of the bagging variables in the environment with the prefix removed
		for VARIABLE in $(env | grep "$VARIABLE_PREFIX$2" | cut -d '=' -f1 | sed "s@$VARIABLE_PREFIX@@"); do

			# Append the variable to the autocomplete list
			COMPREPLY+=( "$VARIABLE" )
		done
	fi
}


bag() {
	local WORKING_DIRECTORY=$PWD
	local MODE="bag"
	local NAME
	local TOPICS
	local ARGS

	# Get the list of bagging variables
	_bagging_complete

	# Handles command line arguments
	while (( $# > 0 )); do
		case $1 in
			-a|--args)
				shift 1
				while (( $# > 0 )); do
					if [[ -z "$ARGS" ]]; then
						ARGS="$1"
					else
						ARGS="$ARGS $1"
					fi
					shift 1
				done
				;;
			-d|--directory)
				export BAG_DIR="$2"
				echo "The bag storage directory is set to $BAG_DIR"
				MODE="false"
				shift 2
				;;
			-g|--group)
				if [[ "$MODE" != "false" ]]; then
					MODE="group"
				fi
				shift 1
				;;
			-h|--help)
				echo "Usage: bag [OPTION]... [BAG_VARIABLE]..."
				echo "Manager for sourcing bagging variables for different MIL vehicles."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-a [ROSBAG_ARG]	--args			Pass arguments after to rosbag"
				echo "-d [DIRECTORY]	--directory		Set the bag storage directory"
				echo "-g 		--group			Group a set of existing variables"
				echo "-h		--help			Display the help menu"
				echo "-l		--list			List available bagging variables"
				echo "-n [BAG_NAME]	--name			Pass a name for bags or groups"
				echo "-s		--show			Show full bag configuration"
				MODE="false"
				shift 1
				;;
			-l|--list)
				if [[ ! -z "$COMPREPLY" ]]; then
					echo "${COMPREPLY[@]}" | sed 's/ /  /g'
				else
					echo "No bagging variables have been loaded"
					echo "Try 'bmux --help' for more information."
				fi
				MODE="false"
				shift 1
				;;
			-n|--name)
				NAME="$2"
				shift 2
				;;
			-s|--show)
				if [[ ! -z "$COMPREPLY" ]]; then
					echo "Bag storage directory:	$BAG_DIR"
					echo "Always bag topics:	$BAG_ALWAYS"
					echo ""
					echo "Bagging variables:"
					env | grep $VARIABLE_PREFIX
				else
					echo "No bagging variables have been loaded"
					echo "Try 'bmux --help' for more information."
				fi
				MODE="false"
				shift 1
				;;
			-*)
				echo "Option $1 is not implemented."
				echo "Try 'bag --help' for more information."
				MODE="false"
				shift 1
				;;
			*)
				if [[ "$MODE" != "false" ]]; then
					if [[ ! -z "$(eval echo \$${VARIABLE_PREFIX}${1})" || "${1:0:1}" == '/' ]]; then
						if [[ -z "$TOPICS" ]]; then
							TOPICS=$(eval echo \$${VARIABLE_PREFIX}${1})
						else
							TOPICS="$TOPICS "$(eval echo \$${VARIABLE_PREFIX}${1})
						fi
					else
						echo "$1 is not one of the available bagging aliases."
						echo "Try 'bag --help' for more information."
						MODE="false"
					fi
				fi
				shift 1
				;;
		esac
	done

	if [[ "$MODE" != "false" ]]; then
		if [[ "$MODE" == "group" && ! -z "$TOPICS" ]]; then
			while [[ -z "$NAME" ]]; do
				echo -n "What should this group be called? " && read NAME
			done
			export "${VARIABLE_PREFIX}${NAME}"="$TOPICS"
		elif [[ "$MODE" == "bag" ]]; then
			while [[ -z "$NAME" ]]; do
				echo -n "What should this bag be called? " && read NAME
			done
			mkdir -p $BAG_DIR"/$(date +%Y-%m-%d)"
			cd $BAG_DIR"/$(date +%Y-%m-%d)"
			rosbag record -O $NAME $ARGS $BAG_ALWAYS $TOPICS
			cd $WORKING_DIRECTORY
		fi
	fi

	unset COMPREPLY
}


# Registers the autocompletion function to be invoked for bag
complete -F _bagging_complete bag
