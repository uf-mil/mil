#!/bin/bash
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
source /opt/ros/melodic/setup.bash
source $MIL_WS/devel/setup.bash

MIL_REPO="$MIL_WS/src/mil"

# Useful aliases
alias mil="cd $MIL_REPO"
alias cm="catkin_make -C $MIL_WS"
