#!/bin/bash
# A variety of bash functions / aliases to network MIL devices together,
# through ROS or otherwise

# Devices on the MIL network start with this IP
MIL_NETWORK_PREFIX="192.168.37"

# Print all devices on the specified subnet / network prefix
list_lan_devices()
{
  if [ $# -lt 1 ]; then
      echo "Usage:   list_lan_devices <subnet>"
      echo "Example: list_lan_devices 192.168.37.1/24"
  fi
  nmap -sP $1 -oG - | awk '/Up$/{print $2}'
}

# List all devices on the MIL network currently by scanning
alias list_mil_devices="list_lan_devices 192.168.37.1/24"

# Get you local IP on the network that you're on
# If on the MIL network and another, chose the MIL network.
# If not on any networks, returns 1
# TODO: verify that "real" networks get priority over docker0 / vpn
get_lan_ip()
{
  IPS=( $(hostname -I) )
  local MIL_IP=$(printf '%s\n' "${IPS[@]}" | grep "$MIL_NETWORK_PREFIX")
  if [ ! -z "$MIL_IP" ]; then
    echo "$MIL_IP"
  elif [ ! -z "$IPS" ]; then
    echo $IPS | awk '{print $1}'
  else
    echo "No local IP addresses. Is WIFI on / Ethernet plugged in?" 1>2&
    return 1
  fi
}

# Setup ros environment variables for local development with no network access
ros_local()
{
  export ROS_IP="127.0.0.1"
  unset ROS_HOSTNAME
  export ROS_MASTER_URI="http://127.0.0.1:11311"
}

# Setup ros environment variables for local development but allowing
# other devices on the network to talk to your roscore
ros_lan()
{
  if ! LAN_IP=$(get_lan_ip) ; then
    ros_local
    return 1
  fi

  export ROS_IP="$LAN_IP"
  unset ROS_HOSTNAME
  export ROS_MASTER_URI="http://$LAN_IP:11311"
}

# Same as ros_lan, but enforces that it is the MIL network and errors otherwise
ros_mil_lan()
{

  ros_lan
  if [[ $(echo $ROS_IP | grep -c "$MIL_NETWORK_PREFIX") == 0 ]]; then
    echo "Not on the MIL network. Reverting to local" 1>&2
    ros_local
    return 1
  fi
}

# Connect to the specified device on the MIL network by IP/domain
# Errors if not on MIL network
# TODO: add option to also specify port of remote roscore, for use
#       when multiple roscores are running on a machine (e.g. zobelisk)
ros_mil()
{
  if ! ros_mil_lan; then
    return 1
  fi

  if [ $# -lt 1 ]; then
      echo "Usage:   ros_mil_lan <ip>"
      echo "Example: ros_mil_lan 192.168.37.60"
  fi

  export ROS_MASTER_URI="http://$1:11311"
}

# Setup ros_connect alias for zobelisk
ZOBELISK_IP="192.168.37.174"
alias ros_zobelisk="ros_mil $ZOBELISK_IP"

# Setup ros connect alias for SubjuGator
SUBJUGATOR_IP="192.168.37.60"
alias ros_sub="ros_mil $SUBJUGATOR_IP"

# All the options you can pass to ros_connect
ROS_CONNECT_OPTIONS="local lan sub mil -h"

# A wrapper around the above functions, allowing
# users to use a single command for ROS networking needs
ros_connect()
{
  if [ $# -lt 1 ] || [ $1 == "-h" ] ; then
      echo "Usage:   ros_connect <mode>"
      echo "Modes:      local - run things entirely on your local machine with no network"
      echo "            lan   - share your ROS network with other devices on the network"
      echo "            mil   - same as lan, but fails if not on MIL network" 
      echo "            sub   - connect your local machine to SubjuGator"
      return 1
  fi

  # Simple call the function corresponding to the mode
  # e.g "ros_connect sub" runs "ros_sub"
  COMMAND="ros_$1"
  $COMMAND
}

# Autocomplete for ROS connect, suggesting all the $ROS_CONNECT_OPTIONS
_ros_connect_complete() {
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "$ROS_CONNECT_OPTIONS" -- ${cur}) )
}
complete -F _ros_connect_complete ros_connect
