# Getting involved

Look at the wiki for the Sub8 github repository. It will walk you through what you need to do and know.

# Setting up the Sub

Sub8 has many dependencies, but there is a convenient install script to fetch and install them. If the default install location of the catkin workspace (~/catkin_ws) is not ideal, then a different path can be passed to the script with the -c option.

#### If you have never cloned this repository or done sub work

The following commands should be run (Don't run as root!):

    wget -O install_sub8.sh https://raw.githubusercontent.com/uf-mil/Sub8/master/install.sh
    chmod +x ./install_sub8.sh
    ./install_sub8.sh -c ~/catkin_ws


#### If you have already cloned the sub or done sub work

The following commands should be run (Don't run as root!):

    ~/catkin_ws/src/Sub8/install.sh -c ~/catkin_ws

Make sure that this is actually the path to the local catkin workspace and install script before running it!

The install script is intended to handle *every single thing* that needs to be installed to run the sub. If it does not work, something has gone wrong and it needs to be fixed. If an issue is fixed while installing, please fix the install script and submit a pull-request with the changes.

The install script can accept arguments. For example, `~/catkin_ws/src/Sub8/install.sh -c ~/mil_ws` will generate a catkin workspace with Sub8 in it at `~/mil_ws` or use an existing workspace if one exists at that location. It will make all of the files and directories it needs within that workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be changed.
