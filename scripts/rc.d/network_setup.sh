#!/bin/bash
MIL_NETWORK_PREFIX="192.168.37"

list_lan_devices()
{
  if [ $# -lt 1 ]; then
      echo "Usage:   list_lan_devices <subnet>"
      echo "Example: list_lan_devices 192.168.37.1/24"
  fi
  nmap -sP $1 -oG - | awk '/Up$/{print $2}'
}

alias list_mil_devices="list_lan_devices 192.168.37.1/24"

get_lan_ip()
{
  IPS=( $(hostname -I) )
  local MIL_IP=$(echo $IPS | grep "$MIL_NETWORK_PREFIX")
  if [ ! -z "$MIL_IP" ]; then
    echo "$MIL_IP"
  elif [ ! -z "$IPS" ]; then
    echo $IPS | awk '{print $1}'
  else
    echo "No local IP addresses. Is WIFI on / Ethernet plugged in?" 1>2&
    return 1
  fi
}
ros_local()
{
  export ROS_IP="127.0.0.1"
  unset ROS_HOSTNAME
  export ROS_MASTER_URI="http://127.0.0.1:11311"
}

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

ros_mil_lan()
{

  ros_lan
  if [[ $(echo $ROS_IP | grep -c "$MIL_NETWORK_PREFIX") == 0 ]]; then
    echo "Not on the MIL network. Reverting to local" 1>&2
    ros_local
    return 1
  fi
}

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

ros_zobelisk()
{
  export ROS_IP="192.168.37.174"
  export ROS_MASTER_URI="http://192.168.37.176:11311"
}

SUBJUGATOR_IP="192.168.37.60"
alias ros_sub="ros_mil $SUBJUGATOR_IP"

ROS_CONNECT_OPTIONS="local lan sub mil -h"

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

  COMMAND="ros_$1"
  $COMMAND
}
_ros_connect_complete() {
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "$ROS_CONNECT_OPTIONS" -- ${cur}) )
}
complete -F _ros_connect_complete ros_connect
