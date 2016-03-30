#**Getting Started**

**Below are the instructions for how to get started working on NaviGator. This page includes instructions and a link to some very helpful readings to start exploring once everything is installed.** 

##Installation

There are three main components to the NaviGator installation: Ubuntu, ROS, and Git. Each of them and their purposes are described below. 

Building the coolest robots can't be done on your grandma's Windows 98 OS. We use some special software and fun tools that require you to ditch the GUI and start using Ubuntu Linux. To communicate on the robot we use a messaging service called ROS. ROS runs on Ubuntu and we run all software with the ROS system. It is a part of the main installation process to get started working on NaviGator. And finally, Git (You are in the wiki right now - [Link to NaviGator GitHub](https://github.com/uf-mil/Navigator)) is the versioning system used to manage all the software used on NaviGator. 
_______________________________________________________________________________________

###Installation instructions

####1. Install Ubuntu


Only Ubuntu 14.04.1 (Trusty) is supported.
The 64-bit version is strongly recommended.

Upgrade an existing install of Ubuntu or download a CD image from 
http://releases.ubuntu.com/trusty/.

Using a VM is not recommended - it requires a much more powerful system 
and causes problems with 3D graphics.
_______________________________________________________________________________________

####2. Install ROS Indigo

Follow all the instructions at 
http://wiki.ros.org/indigo/Installation/Ubuntu. Make sure to do the 
"Desktop-Full Install," as opposed to the "Desktop Install" or 
"ROS-Base."
_______________________________________________________________________________________

####3. Make workspace

Follow all the instructions at 
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment.

BEFORE YOU BEGIN MAKING WORKSPACE - The ROS wiki will advise you to name your 
workspace `/catkin_ws` but this folder name can be named whatever you like. It
is advised to name the folder `/Navigator` instead of `/catkin_ws` and then keep 
all things related to NaviGator and only things related to NaviGator in this
workspace. This will keep your build time low. 

The linked tutorial omits this, but you have to add the following to your
`.bashrc`, after `source /opt/ros/indigo/setup.bash` (which was added during
the tutorial) in order to finish setting up your workspace:

    source ~/WORKSPACE_NAME/devel/setup.bash

After adding those lines to your `.bashrc` file, open a new shell so the
settings take effect.
_______________________________________________________________________________________

####4. Add NaviGator repository

Run:

    roscd && cd ~/WORKSPACE_NAME/src && git clone git@github.com:uf-mil/NaviGator.git

If this fails with a "Permission denied" error, you need to create an SSH key and
add it to your GitHub account.

[Creating SSH Keys] (https://help.github.com/articles/generating-ssh-keys/)

Then, go into the Navigator directory and type `git pull` to pull the newest commits.
_______________________________________________________________________________________


####5. Add SubjuGator repository

Run:

    roscd && cd ~/WORKSPACE_NAME/src && git clone git@github.com:uf-mil/Sub8.git

If this fails with a "Permission denied" error, you need to create an SSH key and
add it to your GitHub account.

[Creating SSH Keys] (https://help.github.com/articles/generating-ssh-keys/)

Then, go into the Sub8 directory and type `git pull` to pull the newest commits.
_______________________________________________________________________________________

####6. Add all NaviGator Dependencies:

All necessary dependencies can be found at the link below.

### [[Software Dependencies]]
_______________________________________________________________________________________

####7. Add all SubjuGator Dependencies:

All necessary dependencies can be installed with the commands below.

    cd ~/catkin_ws/src
    ./Sub8/scripts/get_dependencies.sh -d ~/sub_dependencies
    catkin_make -C ..
_______________________________________________________________________________________


####8. Run catkin_make

Move to your ROS NaviGator workspace and run: 

    catkin_make

If everything was set up properly this should compile everything successfully.
_______________________________________________________________________________________

### **If planning on using the boat while it is in the water, please see the [[Networking with NaviGator]] page**

##Readings and Cool Things to Know:

###[[Helpful Readings]]




  
