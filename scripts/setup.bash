#!/bin/bash
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
source /opt/ros/melodic/setup.bash
source $MIL_WS/devel/setup.bash
source $MIL_WS/src/mil/SubjuGator/scripts/bash_aliases.sh
source $MIL_WS/src/mil/mil_common/ros_alarms/scripts/bash_aliases.sh

MIL_REPO="$MIL_WS/src/mil"

# Script tools
# Helper for autocompleting given a list of choices for the first argument
_list_complete()
{
    local THING
    THINGS=($1)
    for THING in "${THINGS[@]}"; do
            # Skip any entry that does not match the string to complete
            if [[ -z "$2" || ! -z "$(echo ${THING:0:${#2}} | grep $2)" ]]; then
                            COMPREPLY+=( "$THING" )
            fi
    done
}
_list_complete()
{
    local THING
    THINGS=($1)
    for THING in "${THINGS[@]}"; do
            # Skip any entry that does not match the string to complete
            if [[ -z "$2" || ! -z "$(echo ${THING:0:${#2}} | grep $2)" ]]; then
                            COMPREPLY+=( "$THING" )
            fi
    done
}

# Repo aliases
alias mil="cd $MIL_REPO"
alias cm="catkin_make -C $MIL_WS"

# General ROS aliases
ros_env() {
	echo "ROS_IP=$ROS_IP"
	echo "ROS_HOSTNAME=$ROS_HOSTNAME"
	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
}

# Camera helpers
imageproc() # Example usage: imageproc /camera/seecam
{
    ROS_NAMESPACE="$1" rosrun image_proc image_proc
}
calibratecamera()
{
  rosrun camera_calibration cameracalibrator.py --no-service-check --pattern=chessboard --square=0.063 --size=8x6 --disable_calib_cb_fast_check camera:=$1 image:=$1/image_raw
}

# Missions
alias runmission="rosrun mil_missions mission_client run"
alias cancelmission="rosrun mil_missions mission_client cancel"
alias listmissions="rosrun mil_missions mission_client list"
_mission_complete() {
	local MISSION

	# Iterate over the comma diliniated list of known alarms
	for MISSION in $(rosparam get /available_missions | grep -oh "[A-Za-z0-9_ ]*"); do
		# Skip any entry that does not match the string to complete
		if [[ -z "$2" || ! -z "$(echo ${MISSION:0:${#2}} | grep $2)" ]]; then
				COMPREPLY+=( "$MISSION" )
		fi
	done
}


# Registers the autocompletion function to be invoked for ros_connect
complete -F _mission_complete runmission

# Gazebo aliases
alias gazebogui="rosrun gazebo_ros gzclient __name:=gzclient"

# VRX
alias vrxviz="rviz -d \$MIL_REPO/NaviGator/vrx.rviz"

# NaviGator
nthrust()
{
  local topic
  local publishers
  topic="/$1_motor/cmd"
  publishers=$(rostopic info "$topic" | grep Publishers)
  if [ "$publishers" != "Publishers: None" ]; then
     echo "Somone is already publishing to $topic. Perhaps you need to kill thrust mapper?"
     return 1
  fi
  rostopic pub "$topic" "roboteq_msgs/Command" "setpoint: $2" -r100
}
_nthrust_complete()
{
		_list_complete "FL FR BL BR"
}
complete -F _nthrust_complete nthrust
# Tasks
alias nmove="runmission Move"
# Wrench
_nwrench_complete()
{
  _list_complete "rc autonomous keyboard emergency"
}
nwrench()
{
    rosservice call /wrench/select "topic: '$1'"
}
complete -F _nwrench_complete nwrench
# Pneumatics
_nvalve_complete()
{
	# Python oneliners are POWERFULL
  # Get a list of all the actuator names and ids for autocompletion
  ACTUATORS=$(python -c "import rospy; rospy.init_node('test', anonymous=True); param = rospy.get_param('/actuator_driver/actuators'); print ' '.join([key for key in param]) + ' ' + ' '.join([str(param[key]) for key in param if type(param[key]) == int])")

  _list_complete "$ACTUATORS"
}
nvalveopen()
{
  rosservice call /actuator_driver/actuate "'$1'" true
}
complete -F _nvalve_complete nvalveopen

nvalveclose()
{
  rosservice call /actuator_driver/actuate "'$1'" false

}
complete -F _nvalve_complete nvalveclose

nvalvereset()
{
  rosservice call /actuator_driver/reset
}
# Alarms
alias nhold="rosrun ros_alarms raise station-hold"
# Visualization
alias nviz="rviz -d \$MIL_DIR/NaviGator/navigator.rviz"
# Simulation
alias nrespawn="rosservice call /gazebo/delete_model 'navigator' && roslaunch navigator_gazebo spawn_navigator.launch"
# Work around for bagging to SSD and copying to HDD when possible
# Sync bags from the SSD to the HDD
alias nsyncbags="rsync -hru --progress $HOME/bags-fast/ $HOME/bags/"
# Bag to the SSD (so buffer doesn't overflow)
alias nbagfast="BAG_DIR=$HOME/bags-fast bag"
# Clear the bag fast dir on the SSD (run after it is synced)
alias npurgebagfast="rm -r $HOME/bags-fast/*"
complete -F _bagging_complete nbagfast
