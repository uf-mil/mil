sub8 launch files and scripts
=============================


# Nav Box

    roslaunch nav_box.launch dvl:=true imu:=true depth:=true

Each driver can be *turned off* If you are running this launch file alone, each of these defaults to false. This is because the only conditions that I expect someone to run the nav\_box file outside of a launch file is for debugging, and it is thus more convenient to default to off.

This code is almost entirely inherited from previous subs, and is authored almost entirely by Forrest Voight.
