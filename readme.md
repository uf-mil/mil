# Getting involved

Look at the wiki for the Sub8 github repository. It will walk you through what you need to do/know.

# Setting up the Sub

Sub8 has many dependencies, but we have a convenient install script. If you don't like where the dependencies are automatically installed (repos/sub8_dependencies), or where the catkin workspace is made (~/repos/catkin_ws), then you can change the variables set in the script.

#### If you have never downloaded this repository or set up ROS

I suggest you do the following (Don't run as sudo!):

    wget -O install_sub8.sh https://raw.githubusercontent.com/uf-mil/Sub8/master/install.sh
    chmod +x ./install_sub8.sh
    ./install_sub8.sh -d ~/repos/sub_dependencies -c ~/catkin_ws
    cd ~/catkin_ws
    catkin_make


#### If you have already cloned the sub (Or done sub work)

You can just do (Don't run as sudo!)

    cd ~/catkin_ws/src
    ./Sub8/scripts/get_dependencies.sh -d ~/sub_dependencies
    catkin_make -C ..

In the Sub8 root directory. It will check if you have dependencies installed and *not install them* if you already have them

The install script is intended to handle *every single thing* that needs to be installed to run the sub. If it does not work, something has gone wrong that we need to fix. If you fix an issue while installing, please fix the install script and submit a pull-request with your changes.

The install script can accept arguments. `./install ~/repos/deps ~/mil_ws` will install the sub dependencies in `~/repos/deps` and generate a catkin workspace with Sub8 in it at `~/mil_ws`. It will make all of the directories it needs