#NaviGator Autonomous Surface Vehicle

####Designed by the University of Florida for the [AUVSI Maritime RobotX Challenge](http://www.robotx.org)
#####This is the repository for UF's NaviGator ASV. It contains all software packages specific to NaviGator.

# Getting involved

[**JOIN THE GOOGLE GROUP!**] (https://groups.google.com/forum/#!forum/uf-navigator-asv)

Click the link above and find where it says to subscribe. All communication goes through Git and the group. 

To get started working with the software on NaviGator please contact Zach Goins at:

    zach.a.goins@gmail.com
    918.801.8242

**Let's go eat food and talk about how cool robots are.**

Peruse the repository, check out some code. This [wiki](https://github.com/uf-mil/Navigator/wiki) is your master guide for both those things.

# Setting Up the Development Environment

NaviGator has many dependencies, but there is a convenient install script to fetch and install them. If the default install location of the catkin workspace (~/mil_ws) is not ideal, then a different path can be passed to the script with the -c option.

#### If you have never cloned this repository or done work in MIL

The following commands should be run (Don't run as root!):

    wget -O mil_install.sh https://raw.githubusercontent.com/uf-mil/Navigator/master/install.sh
    chmod +x ./mil_install.sh
    ./mil_install.sh -n


#### If you have already cloned this repository or done work in MIL

The following commands should be run (Don't run as root!):

    ~/mil_ws/src/Navigator/install.sh

Make sure that this is actually the path to the local catkin workspace and install script before running it!

The install script is intended to handle *every single thing* that needs to be installed to develop for NaviGator. If it does not work, something has gone wrong and it needs to be fixed. If an issue is fixed while installing, please fix the install script and submit a pull-request with the changes.

The install script can accept arguments. For example, `./mil_install.sh -c ~/navigator_ws` will generate a catkin workspace with the NaviGator repository in it at `~/navigator_ws` or use an existing workspace if one exists at that location. It will make all of the files and directories it needs within that workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be changed.

[![Build Status](https://semaphoreci.com/api/v1/uf-mil/navigator-2/branches/pull-request-30/badge.svg)](https://semaphoreci.com/uf-mil/navigator-2)
