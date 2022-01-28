#!/bin/bash
#switches from the vrx_2022 branch to the master branch and compiles

export MIL_CONFIG_DIR=$HOME/.mil
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
MIL_REPO="$MIL_WS/src/mil"

catkin_make -C $MIL_WS

#removes all dock_2022 files in vrx_gazebo
if find ~/catkin_ws/src/mil/NaviGator/simulation/VRX/vrx/vrx_gazebo/models -type d -name 'dock_2022*' -exec false {} +
then
	echo "no dock_2022 files found"
else
	find ~/catkin_ws/src/mil/NaviGator/simulation/VRX/vrx/vrx_gazebo/models -type d -name 'dock_2022*' -prune -print -printf "dock_2022 files removed\n" -exec rm -rf {} +
fi

git checkout master
git submodule update --init --recursive
echo "script complete"
