# sabertooth2x12
This is a ROS driver for the sabertooth2x12 motor controller. It provides two topics:

`motor1_cmd/`
`motor2_cmd/`


Which are both `std_msgs/Float64` type and set the value of the motor controller based on a float
from -1.0 (full backwards) to 1.0 (full forwards).


# Usage
Launch with `roslaunch sabertooth2x12 example.launch`. You may need to change the port
parameter in the launch file to be the correct USB device.

# Simulation
If run with the ROS param `/is_simulation` set to true, it will simulate the serial commands on the board.
