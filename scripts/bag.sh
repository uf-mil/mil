#!/bin/bash

# The bag command is mainly a convenience layer for rosbag; however, it does
# introduce a few features. A list of BAG_ALWAYS topics can include topics that
# every bag should contain, bag directory management is handled cleanly, and
# naming bags is... strongly encouraged. The configuration can be imported from
# and exported to a file, meaning the bagging variables can be version
# controlled for each project. Once a configuration file is loaded, bagging
# variables will be autocompleted by bash and can be grouped on the fly.


CONFIGURATION_FILE="bagging_variables.sh"
VARIABLE_PREFIX="bag_"


_bagging_complete() {
	local ITEM
	if [[ ! -z "$COMP_CWORD" ]]; then
		local PREVIOUS="${COMP_WORDS[$((COMP_CWORD - 1))]}"
	fi

	# If the previous argument signifies an import, complete from configuration files
	if [[ "$PREVIOUS" == "-i" || "$PREVIOUS" == "--import" ]]; then

		# Check for configuration files in the catkin workspace repositories
		for ITEM in $CATKIN_DIR/src/"$2"*; do

			# Skip any repository that does not contain a configuration file
			if [[ -f $ITEM/scripts/$CONFIGURATION_FILE ]]; then

				# Append just the name of the repository to the autocomplete list
				COMPREPLY+=( $(echo "$ITEM" | rev | cut -d '/' -f1 | rev) )
			fi
		done

	# If the previous argument signifies an export, complete from repository scripts directories
	elif [[ "$PREVIOUS" == "-e" || "$PREVIOUS" == "--export" ]]; then

		# Check for scripts directories in the catkin workspace repositories
		for ITEM in $CATKIN_DIR/src/"$2"*; do

			# Skip any repository that does not contain a scripts directories
			if [[ -d $ITEM/scripts ]]; then

				# Append just the name of the repository to the autocomplete list
				COMPREPLY+=( $(echo "$ITEM" | rev | cut -d '/' -f1 | rev) )
			fi
		done

	# Append all ROS topics to the autocomplete list if the string to autocomplete begins with a '/' character
	elif [[ "${2:0:1}" == '/' ]]; then
		COMPREPLY+=( $(rostopic list 2> /dev/null) )

	else

		# Otherwise, iterate over all of the bagging variables in the environment with the prefix removed
		for ITEM in $(env | grep "$VARIABLE_PREFIX$2" | cut -d '=' -f1 | sed "s@$VARIABLE_PREFIX@@"); do

			# Append the variable to the autocomplete list
			COMPREPLY+=( "$ITEM" )
		done
	fi
}

generate_configuration() {
	local BAGGING_VARIABLES="$(env | grep 'bag_' | cut -d '=' -f1 | cut -d ' ' -f2)"
	local VARIABLE

	echo "#!/bin/bash"
	echo ""
	echo "# This script will be sourced by the bag command as a configuration file. It is"
	echo "# imperative that each variable is in the bag_* namespace and contains a space"
	echo "# deliniated list of ROS topics. The only two exceptions to this rule are the"
	echo "# BAG_ALWAYS and BAG_DIR variables."
	echo ""
	echo ""
	echo "# Define topics that should be in every bag"
	echo "export BAG_ALWAYS=\"$BAG_ALWAYS\""
	echo ""
	echo "# Define the directory that the bags will be stored in"
	echo "export BAG_DIR=$BAG_DIR"
	echo ""
	echo ""
	echo "# Topic variables that can be used from the bag command"

	for VARIABLE in $BAGGING_VARIABLES; do
		echo "export $VARIABLE=\"$(env | grep $VARIABLE= | cut -d '=' -f2)\""
	done
}

bag() {
	local BAGGING_VARIABLES="$(env | grep 'bag_' | cut -d '=' -f1 | cut -d ' ' -f2)"
	local WORKING_DIRECTORY=$PWD
	local NOTE_TEXT="false"
	local MODE="bag"
	local TOPICS
	local NAME
	local TIME
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
			-c|--clear)
				if [[ ! -z "$BAGGING_VARIABLES" ]]; then
					for VARIABLE in $BAGGING_VARIABLES; do
						unset "$VARIABLE"
					done
				fi

				# Unset special variables separately
				unset BAG_ALWAYS
				unset BAG_DIR
				MODE="false"
				shift 1
				;;
			-d|--directory)
				export BAG_DIR="$2"
				echo "The bag storage directory is set to $BAG_DIR"
				MODE="false"
				shift 2
				;;
			-e|--export)
				if [[ -d $CATKIN_DIR/src/$2/scripts ]]; then
					generate_configuration > $CATKIN_DIR/src/$2/scripts/$CONFIGURATION_FILE

				# Inform the user if the selected repository does not have a scripts directory
				else
					echo "$CATKIN_DIR/src/$2 has no scripts directory."
					echo "Try 'bag --help' for more information."
				fi
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
				echo "Usage: bag [OPTION]... [BAG_VARS]..."
				echo "Wrapper for efficiently using the rosbag command."
				echo ""
				echo "Option		GNU long option		Meaning"
				echo "-a [ROSBAG_ARG]	--args			Pass arguments after to rosbag"
				echo "-c		--clear			Clear the existing configuration"
				echo "-d [DIRECTORY]	--directory		Set the bag storage directory"
				echo "-e [REPOSITORY]	--export		Export the configuration to a file"
				echo "-g [BAG_VARS]	--group			Group existing variables and topics"
				echo "-h		--help			Display the help menu"
				echo "-i [REPOSITORY]	--import		Import a configuration from a file"
				echo "-l		--list			List available bagging variables"
				echo "-n [BAG_NAME]	--name			Pass a name for bags or groups"
				echo "-o [TIME]	--online		Bag a number of seconds in the past"
				echo "-s		--show			Show full bag configuration"
				echo "-t		--text			Open a text file for taking notes"
				MODE="false"
				shift 1
				;;
			-i|--import)

				# Clear existing bagging aliases before importiing new ones
				bag -c

				if [[ -f $CATKIN_DIR/src/$2/scripts/$CONFIGURATION_FILE ]]; then
					source $CATKIN_DIR/src/$2/scripts/$CONFIGURATION_FILE

				# Inform the user if the selected file does not exist
				else
					echo "$CATKIN_DIR/src/$2 has no scripts/bagging_variables.sh file."
					echo "Try 'bag --help' for more information."
				fi
				MODE="false"
				shift 1
				;;
			-l|--list)
				if [[ ! -z "$COMPREPLY" ]]; then
					echo "${COMPREPLY[@]}" | sed 's/ /  /g'
				else
					echo "No bagging variables have been loaded"
					echo "Try 'bag --help' for more information."
				fi
				MODE="false"
				shift 1
				;;
			-n|--name)
				NAME="$2"
				shift 2
				;;
			-o|--online)
				if [[ "$MODE" != "false" ]]; then
					MODE="online"
					TIME="$2"
				fi
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
					echo "Try 'bag --help' for more information."
				fi
				MODE="false"
				shift 1
				;;
			-t|--text)
				NOTE_TEXT="true"
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

	# If grouping mode was selected, create a new bagging variable
	if [[ "$MODE" == "group" && ! -z "$TOPICS" ]]; then

		# Retrieve a variable name if one was not passed
		while [[ -z "$NAME" ]]; do
			echo -n "What should this group be called? " && read NAME
		done
		export "${VARIABLE_PREFIX}${NAME}"="$TOPICS"

	# If bagging mode was selected, create a ROS bag
	elif [[ "$MODE" == "bag" ]]; then

		# Retrieve a bag name if one was not passed
		while [[ -z "$NAME" ]]; do
			echo -n "What should this bag be called? " && read NAME
		done

		# Store the bag in the correct dated folder
		mkdir -p $BAG_DIR"/$(date +%Y-%m-%d)"
		cd $BAG_DIR"/$(date +%Y-%m-%d)"
		rosbag record -O $NAME $ARGS $BAG_ALWAYS $TOPICS

		# If the text flag was passed in, create a notes file of the same name
		if [[ "$NOTE_TEXT" == "true" ]]; then
			vim $NAME.txt
		fi

		# Return the user to the directory they ran the command from
		cd $WORKING_DIRECTORY

	# If online mode was selected, call the online bagger service
	elif [[ "$MODE" == "online" ]]; then

		# Retrieve a bag name if one was not passed
		while [[ -z "$NAME" ]]; do
			echo -n "What should this bag be called? " && read NAME
		done

		# Call to the online bagger with what topics to dump and where
		rosservice call /online_bagger/dump "{bag_name: '$BAG_DIR/$(date +%Y-%m-%d)/$NAME', topics: '$TOPICS', bag_time: '$TIME'}"

		# If the text flag was passed in, create a notes file of the same name
		if [[ "$NOTE_TEXT" == "true" ]]; then
			vim $BAG_DIR"/$(date +%Y-%m-%d)"/$NAME.txt
		fi
	fi

	unset COMPREPLY
}


# Registers the autocompletion function to be invoked for bag
complete -F _bagging_complete bag
