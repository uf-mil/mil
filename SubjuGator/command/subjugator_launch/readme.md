sub8 launch files and scripts
=============================


# Nav Box

    roslaunch nav_box.launch dvl:=true imu:=true depth:=true

Each driver can be *turned off* If you are running this launch file alone, each of these defaults to false. This is because the only conditions that I expect someone to run the nav\_box file outside of a launch file is for debugging, and it is thus more convenient to default to off.

This code is almost entirely inherited from previous subs, and is authored almost entirely by Forrest Voight.

# PID Debugging Data Collection

**NOTE:** This launch file needs the sub launch file (`sub8.launch`) running in order to function properly.

    roslaunch subjugator_launch bag_debugging_controller.launch prefix_name:="my_prefix"

This launch file records a bag file containing adaptive controller `pose_error`, `twist_error`, `adaptation`, `dist`, and `drag` data along with `wrench`, `trajectory`, `odom`, `imu/data_raw`, and `dvl/range` data. This data is useful in determining if the PID values are getting better or worse using simulations. The bag file was written by Andres Pulido.
