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

You'll also need pyode

# Usage

run
    
    roslaunch sub8_simulation sim_full.launch


# Truth

Truth odometry is published on /truth/odom

You can set the truth pose by calling the service /sim/vehicle/set_pose using the sub8_simulation.srv.SimSetPose service type
