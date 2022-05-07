#!/bin/sh

#find where the 'sjtu_drone' is
pack_path=$(rospack find sjtu_drone)

#export the gazebo pathes
export GAZEBO_MODEL_PATH=$pack_path/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$pack_path:/usr/share/gazebo-3.0:/usr/share/gazebo-4.0:/usr/share/gazebo_models:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=$pack_path/plugins:$GAZEBO_PLUGIN_PATH

#call the client of Gazebo
roslaunch drone_construct main.launch gui:=true paused:=false
