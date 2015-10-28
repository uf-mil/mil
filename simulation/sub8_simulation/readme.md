Sub8 Simulation
===============

See the wiki for a more in-depth explanation of the inner workings of the simulator

# Installation

You need to install Vispy from source, do this..

    mkdir repos
    cd repos
    git clone https://github.com/vispy/vispy.git
    cd vispy
    git checkout b48e4d3cf410b853a74b666c475c603e46725e55
    sudo python setup.py develop

You'll also need pyode.

# Usage

run

    roslaunch sub8_simulation sim_full.launch

To use with the 3D mouse, sending wrenches, do

    roslaunch sub8_simulation sim_full.launch 3dmouse:=True

To set the speed higher, then in the roslaunch you'll have to set the `time_acceleration` parameter like this

    <param name="time_acceleration" value="60" type="double"/>
    <param name="draw" value=False />

The `time_acceleration` is the number of simulated seconds that should pass for every real-world second. In this case, that's one simulated minute per real second. In *general*, though it is not guaranteed, if the sim runs slower than the acceleration you've asked for, time will slow down accordingly and everything will move smoothly.

Note: For time-acceleration greater than 1.5, you must disable rendering, by setting the `draw` parameter to false

# Truth

Truth odometry is published on /truth/odom

You can set the truth pose by calling the service /sim/vehicle/set_pose using the sub8_simulation.srv.SimSetPose service type

