# Getting involved

Look at the wiki for the Sub8 github repository. It will walk you through what you need to do/know.

# Setting up the Sub

Sub8 has many dependencies, but we have a convenient install script. If you don't like where the catkin workspace is made (~/repos/catkin_ws), then you can pass a different path to the script with the -c option.

#### If you have never downloaded this repository or set up ROS

I suggest you do the following (Don't run as root!):

    wget -O install_sub8.sh https://raw.githubusercontent.com/uf-mil/Sub8/master/install.sh
    chmod +x ./install_sub8.sh
    ./install_sub8.sh -c ~/catkin_ws


#### If you have already cloned the sub (Or done sub work)

You can just do the following (Don't run as root!):

    ~/catkin_ws/src/Sub8/install.sh -c ~/catkin_ws

Make sure that this is actually the path to your catkin workspace and install script before running it!

The install script is intended to handle *every single thing* that needs to be installed to run the sub. If it does not work, something has gone wrong that we need to fix. If you fix an issue while installing, please fix the install script and submit a pull-request with your changes.

The install script can accept arguments. `~/catkin_ws/src/Sub8/install.sh -c ~/mil_ws` will generate a catkin workspace with Sub8 in it at `~/mil_ws` or use an existing workspace at that location. It will make all of the files and directories it needs within that workspace. If you have previously run the script, it should not run initial set up tasks that have already been performed unless they need to be changed.
