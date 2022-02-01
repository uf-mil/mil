#!/bin/bash
#switches from the vrx_2022 branch to the master branch and compiles

export MIL_CONFIG_DIR=$HOME/.mil
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
MIL_REPO="$MIL_WS/src/mil"

git checkout master

git submodule update --init --recursive

#removes the navigator_vrx.urdf file in navigator_gazebo
FILE=~/catkin_ws/devel/share/navigator_gazebo/urdf/navigator_vrx.urdf
if test -f "$FILE"; then
	rm ~/catkin_ws/devel/share/navigator_gazebo/urdf/navigator_vrx.urdf
	echo "removed navigator_vrx.urdf"
else
	echo "no navigator_vrx.urdf to remove"
fi

#removes all dock_2022 files in vrx_gazebo
if find ~/catkin_ws/src/mil/NaviGator/simulation/VRX/vrx/vrx_gazebo/models -type d -name 'dock_2022*' -exec false {} +
then
	echo "no dock_2022 files found"
else
	find ~/catkin_ws/src/mil/NaviGator/simulation/VRX/vrx/vrx_gazebo/models -type d -name 'dock_2022*' -prune -print -printf "dock_2022 files removed\n" -exec rm -rf {} +
fi

catkin_make -C $MIL_WS

echo "script complete"
