#!/bin/bash

# This file contains custom bash alias commands for MIL projects. It is copied
# from scripts/mil_aliases.sh in the Navigator repository when the install
# script is run, so any changes made to it will be overwritten.

# These are the hostnames for all devices that run a remote roscore
SUB_HOST=mil-sub-sub8.ad.mil.ufl.edu
WMV_HOST=mil-nav-wamv.ad.mil.ufl.edu
PRC_HOST=mil-nav-perception.ad.mil.ufl.edu
SHT_HOST=mil-shuttle.ad.mil.ufl.edu
JN5_HOST=mil-johnny-five.ad.mil.ufl.edu

check_host() {

	# Attempts to ping a host to make sure it is reachable
	HOST="$1"

	HOST_PING=$(ping -w 1 -c 2 $HOST 2>&1 | grep "% packet" | cut -d" " -f 6 | tr -d "%")
	if ! [ -z "${HOST_PING}" ]; then

		# Uses packet loss percentage to determine if the connection is strong
		if [ $HOST_PING -lt 25 ]; then

			# Will return true if ping was successful and packet loss was below 25%
			return `true`
		fi
	fi
	return `false`
}

check_connection() {
	if (check_host "$SUB_HOST"); then
		SUB_CHECK=true
	else
		SUB_CHECK=false
	fi
	if (check_host "$WMV_HOST"); then
		WMV_CHECK=true
	else
		WMV_CHECK=false
	fi
	if (check_host "$PRC_HOST"); then
		PRC_CHECK=true
	else
		PRC_CHECK=false
	fi
	if (check_host "$SHT_HOST"); then
		SHT_CHECK=true
	else
		SHT_CHECK=false
	fi
	if (check_host "$JN5_HOST"); then
		JN5_CHECK=true
	else
		JN5_CHECK=false
	fi

	if ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "false" ] &&\
	    [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "false" ] &&\
	    [ "$JN5_CHECK" = "false" ]); then
		echo "None of the MIL roscores are available on this network"
	else
		if [ "$SUB_CHECK" = "true" ]; then
			echo "SubjuGator is accessible on this network"
		fi
		if [ "$WMV_CHECK" = "true" ]; then
			echo "Navigator WAMV is accessible on this network"
		fi
		if [ "$PRC_CHECK" = "true" ]; then
			echo "Navigator Perception is accessible on this network"
		fi
		if [ "$SHT_CHECK" = "true" ]; then
			echo "Shuttle is accessible on this network"
		fi
		if [ "$JN5_CHECK" = "true" ]; then
			echo "Johnny Five is accessible on this network"
		fi
	fi
}

set_ros_ip() {
	LOCAL_IP="`ip route get 192.168.37.0/24 | awk '{print $NF; exit}'`"
	LOCAL_HOSTNAME="`hostname`.ad.mil.ufl.edu"

	# Sets ROS_HOSTNAME to the this machine's hostname
	export ROS_HOSTNAME=$LOCAL_HOSTNAME

	# Sets ROS_IP to the IP on this machine's main NIC
	export ROS_IP=$LOCAL_IP
}

unset_ros_ip() {

	# Unsets ROS_IP and ROS_HOSTNAME, which will default to localhost
	unset ROS_IP
	unset ROS_HOSTNAME
}

set_ros_master() {

	# Sets ROS_MASTER_URI to the hostname of the correct MIL roscore
	export ROS_MASTER_URI=$REMOTE_ROSCORE_URI
	echo "The master roscore is set to $REMOTE_ROSCORE_URI"
}

unset_ros_master() {

	# Sets ROS_MASTER_URI to point back to localhost
	export ROS_MASTER_URI=http://localhost:11311
	echo "The master roscore is set to this machine"
}

ros_connect() {
	check_connection

	# If none of the MIL roscores are accessible, use localhost as the default roscore
	if ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "false" ] &&\
	    [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "false" ] &&\
	    [ "$JN5_CHECK" = "false" ]); then
		ros_disconnect

	# If just one MIL roscore was accessible, connect directly to that roscore
	elif ([ "$SUB_CHECK" = "true" ] && [ "$WMV_CHECK" = "false" ] &&\
	      [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "false" ] &&\
	      [ "$JN5_CHECK" = "false" ]); then
		REMOTE_ROSCORE_URI=http://$SUB_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "true" ] &&\
	      [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "false" ] &&\
	      [ "$JN5_CHECK" = "false" ]); then
		REMOTE_ROSCORE_URI=http://$WMV_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "false" ] &&\
	      [ "$PRC_CHECK" = "true" ] && [ "$SHT_CHECK" = "false" ] &&\
	      [ "$JN5_CHECK" = "false" ]); then
		REMOTE_ROSCORE_URI=http://$PRC_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "false" ] &&\
	      [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "true" ] &&\
	      [ "$JN5_CHECK" = "false" ]); then
		REMOTE_ROSCORE_URI=http://$SHT_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ "$SUB_CHECK" = "false" ] && [ "$WMV_CHECK" = "false" ] &&\
	      [ "$PRC_CHECK" = "false" ] && [ "$SHT_CHECK" = "false" ] &&\
	      [ "$JN5_CHECK" = "true" ]); then
		REMOTE_ROSCORE_URI=http://$JN5_HOST:11311
		set_ros_ip
		set_ros_master

	# If multiple roscores were reachable, allow the user to select one
	else
		echo ""
		echo "Multiple MIL roscores were detected!"
		if ([ "$SUB_CHECK" = "true" ]); then
			echo "	1. SubjuGator"
		fi
		if ([ "$WMV_CHECK" = "true" ]); then
			echo "	2. Navigator WAMV"
		fi
		if ([ "$PRC_CHECK" = "true" ]); then
			echo "	3. Navigator Perception"
		fi
		if ([ "$SHT_CHECK" = "true" ]); then
			echo "	4. Shuttle"
		fi
		if ([ "$JN5_CHECK" = "true" ]); then
			echo "	5. Johnny Five"
		fi
		echo ""
		echo -n "Select a roscore to connect to: "
		read SELECTION

		if [ "$SELECTION" = "1" ]; then
			REMOTE_ROSCORE_URI=http://$SUB_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ "$SELECTION" = "2" ]; then
			REMOTE_ROSCORE_URI=http://$WMV_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ "$SELECTION" = "3" ]; then
			REMOTE_ROSCORE_URI=http://$PRC_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ "$SELECTION" = "4" ]; then
			REMOTE_ROSCORE_URI=http://$SHT_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ "$SELECTION" = "5" ]; then
			REMOTE_ROSCORE_URI=http://$JN5_HOST:11311
			set_ros_ip
			set_ros_master
		else
			echo "Invalid selection value, no roscore selected"
		fi
	fi
}

ros_disconnect() {

	# Disconnects from any remote roscore and connects to the local one
	unset_ros_ip
	unset_ros_master
}

# Prints debugging output for the master roscore that is currently selected
alias rosenv='echo "ROS_IP=$ROS_IP
ROS_HOSTNAME=$ROS_HOSTNAME
ROS_MASTER_URI=$ROS_MASTER_URI"'
