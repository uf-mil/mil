#!/bin/bash

# This script will be sourced by the MIL runcom file if the install script
# cloned this repository during a run or if the install script was run after
# it was cloned manually. It contains NaviGator specific commands that have
# been aliased to make them faster or easier to execute. Becoming familiar with
# these will likely increase productivity, so it is recommended to do so.


# Directory navigation
alias nav="cd $CATKIN_DIR/src/NaviGator"

# Networking
alias rnav="ros_connect -n ${HOSTNAMES[1]}"
alias sshnav="ssh navigator@${HOSTNAMES[1]} -Y"

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
alias nmove="runtask Move"

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
alias nviz="rviz -d \$CATKIN_DIR/src/NaviGator/navigator.rviz"

# Development
alias navfmt="python2.7 -m flake8 --ignore E731 --max-line-length=120 \$(rosrun mil_tools list_python_files \$CATKIN_DIR/src/NaviGator __init__.py deprecated/ gnc/navigator_path_planner/lqRRT .cfg simulation/vmrc)"
alias navtest="(cd \$CATKIN_DIR; rosrun mil_tools catkin_tests_directory.py src/NaviGator)"

# Simulation
alias nrespawn="rosservice call /gazebo/delete_model 'navigator' && roslaunch navigator_gazebo spawn_navigator.launch"

# Work around for bagging to SSD and copying to HDD when possible
# Sync bags from the SSD to the HDD
alias syncbags="rsync -hru --progress $HOME/bags-fast/ $HOME/bags/"
# Bag to the SSD (so buffer doesn't overflow)
alias bagfast="BAG_DIR=$HOME/bags-fast bag"
# Clear the bag fast dir on the SSD (run after it is synced)
alias purgebagfast="rm -r $HOME/bags-fast/*"
complete -F _bagging_complete bagfast
