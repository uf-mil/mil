#! /usr/bin/env bash

## Ensure that the user is at ~/catkin_ws/src/mil
cd ~/catkin_ws/src/mil || exit

cmonly() {
	cd ~/catkin_ws || exit
	catkin_make --only-pkg-with-deps "$1"
	cd - >/dev/null || exit
}

## Clone the magical_ros2 repository
git clone https://github.com/DLu/roscompile
cd roscompile || exit
cmonly ros_introspection
cmonly roscompile
cmonly magical_ros2_conversion_tool
cd ~/catkin_ws/src/mil || exit

python3 -m pip install ruamel.yaml

echo -e "\n\n\n\n\n"
echo "To call the conversion tool, run the following command:"
echo ">>> rosrun magical_ros2_conversion_tool ros2_conversion"
echo "This will convert the ROS1 packages in the workspace to ROS2 packages."
