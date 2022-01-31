#!/bin/bash
#switches from the master branch to the vrx_2022 branch and compiles

#There is an error in this file. Adds a second parameter to the file yaml.load (,Loader=yaml.Loader)
#sudo sed -i 's/values_dict = yaml.load(value)/values_dict = yaml.load(value, Loader-yaml.Loader)/g' /opt/ros/melodic/lib/dynamic_reconfigure/dynparam

export MIL_CONFIG_DIR=$HOME/.mil
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
MIL_REPO="$MIL_WS/src/mil"

catkin_make -C $MIL_WS

#removes the navigator_vrx.urdf file in navigator_gazebo
FILE=~/catkin_ws/devel/share/navigator_gazebo/urdf/navigator_vrx.urdf
if test -f "$FILE"; then
	rm ~/catkin_ws/devel/share/navigator_gazebo/urdf/navigator_vrx.urdf
	echo "removed navigator_vrx.urdf"
else
	echo "no navigator_vrx.urdf to remove"
fi

git checkout vrx_2022
git submodule update --init --recursive

echo "script complete"
