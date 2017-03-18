# Getting Involved

Information about getting involved in MIL is available on the [mil_common wiki](https://github.com/uf-mil/mil_common/wiki). The home page will introduce you to the various projects that are under development in the lab and provide a link to the getting started guide.

# Setting Up the Development Environment

If you have decided to develop for one of our projects, then you will need to set up a development environment that is compatible with it. All of the projects have many dependencies, but there is a convenient install script in this repository to fetch and install them. The script covers all MIL projects that are currently on Ubuntu 16.04 / Debian 8.7 and ROS Kinetic and will allow you to select which one(s) you would like to set up. If the default install location of the catkin workspace at `~/mil_ws` is not ideal, then a different path can be selected when the script prompts for one.

This repository does not need to be cloned locally unless you want to make changes to it. The following command will fetch and run the script:

    sudo apt-get -qq update && sudo apt-get install -qq curl && bash <(curl -s https://raw.githubusercontent.com/uf-mil/installer/master/install.sh)

The install script is intended to handle *every single thing* that needs to be installed to develop for the robot. If it does not work, something has gone terribly wrong and it needs to be fixed. If you resolve an issue while installing, please fix it in the install script and submit a pull-request with the changes. Otherwise, notify the script's maintainer (currently [Anthony Olive](https://github.com/whispercoros)).

The script will create all of the files and directories it needs within the selected catkin workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be updated. This means that the script will respect workspaces with git repositories already present in them.
