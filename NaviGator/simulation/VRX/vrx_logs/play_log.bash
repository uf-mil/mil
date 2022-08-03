#! /bin/bash
set -euo pipefail

usage()
{
  echo "Usage: $0  <year> <task> <run>"
}

if [ $# -lt 3 ] ; then
  usage
fi

CURRENT_DIR="$(realpath $(dirname $BASH_SOURCE))"
YEAR=$1
TASK=$2
RUN=$3

VIDEO_FILE=${4:-""}

LOG_FILE=$CURRENT_DIR/$YEAR/uf/$TASK/$RUN/gazebo-server/state.log

roslaunch vrx_gazebo playback.launch \
  log_file:=$LOG_FILE \
  verbose:=true \
