#!/bin/bash
export MIL_WS="$(realpath $(dirname $BASH_SOURCE)/../../..)"
source /opt/ros/melodic/setup.bash
source $MIL_WS/devel/setup.bash

MIL_REPO="$MIL_WS/src/mil"
alias mil="cd $MIL_REPO"
