# AUVSI Maritime RobotX Challenge

##RobotX Wam-V on board softare git
==========

This is the repository for the University of Florida's RobotX Autonomous Robot. It contains all ROS packages specifically to RobotX
## Installation instructions

1. Install Ubuntu
-----------------

Only Ubuntu 14.04.1 (Trusty) is supported.
The 64-bit version is strongly recommended.

Upgrade an existing install of Ubuntu or download a CD image from 
http://releases.ubuntu.com/trusty/.

Using a VM is not recommended - it requires a much more powerful system 
and causes problems with 3D graphics.

2. Install ROS Indigo
---------------------

Follow all the instructions at 
http://wiki.ros.org/indigo/Installation/Ubuntu. Make sure to do the 
"Desktop-Full Install," as opposed to the "Desktop Install" or 
"ROS-Base."

3. Make workspace
-----------------

Follow all the instructions at 
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment.

BEFORE YOU BEGIN MAKING WORKSPACE - The ROS wiki will advise you to name your 
workspace `/catkin_ws` but this folder name can be named whatever you like. It
is advised to name the folder `/RobotX` instead of `/catkin_ws` and then keep 
all things related to RobotX and only things related to RobotX in this
workspace. This will keep your build time low. 

The linked tutorial omits this, but you have to add the following to your
`.bashrc`, after `source /opt/ros/indigo/setup.bash` (which was added during
the tutorial) in order to finish setting up your workspace:

    source ~/WORKSPACE_NAME/devel/setup.bash

After adding those lines to your `.bashrc` file, open a new shell so the
settings take effect.

4. Add RobotX repository
--------------------------

Run:

    roscd && cd ~WORKSPACE_NAME/src && git clone git@github.com:uf-mil/RobotX.git

If this fails with a "Permission denied" error, you need to create an SSH key and
add it to your GitHub account.

[Creating SSH Keys] (https://help.github.com/articles/generating-ssh-keys/)

Then, go into the RobotX directory and type `git pull` to pull the newest commits.

5. Run catkin_make
------------------

Move to your ROS RobotX workspace and run: 

    catkin_make

If everything was set up properly this should compile everything successfully

  
