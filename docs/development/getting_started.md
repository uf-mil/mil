# Getting Started
This page describes the recommended process of setting up your new ubuntu system with the repository and its dependencies.

## System Requirements

### Operating System

**It is recommended that you [dual boot Ubuntu 18.04](https://help.ubuntu.com/community/WindowsDualBoot) for development**

Virtual Machines will be able to run our software, but it may be too slow (see [hardware](#hardware))

### Hardware

Autonomous robotics is computationally expensive, especially when running simulations.
If you can, use a powerful computer. We recommend you have at least 8GB RAM and a modern i5 or better CPU. Many tasks such as simulation and computer vision will run faster on a system with a GPU.

## Starting from a clean slate

First, we will want to check if there are any updates. Sometimes this is already done for you assuming you connected to internet during the ubuntu system setup guide.

`sudo apt update`

`sudo apt upgrade`

## Install git

To contribute changes, you will need to have a git client installed. This program will be used to track and upload your changes.

`sudo apt install git`

## Cloning the repository

You need to clone (download) a copy of the repository onto your computer so you can make changes.

First clone the upstream (MIL's fork) version of the repo.
It is recommended that you first create a catkin workspace
and clone it into the `src` or that workspace.

`mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`

Then clone the repo:

(the `--recurse-submodules` is essential, as that is what pulls the submodules, `-j8` is a speed optimization that downloads up to 8 submodules at a time)

`git clone --recurse-submodules -j8 https://github.com/uf-mil/mil.git`

**Navigate to the root of the repository now.**

If you have followed the instructions exactly, you can get there by running

`cd ~/catkin_ws/src/mil`

### Run the setup scripts

If you are running Ubuntu 18.04 and prefer to simply run code directly on your "host"
operating system without using Docker, you can run the following scripts from the root of the repository

`./scripts/system_install`

`./scripts/user_install`

Exit the terminal and enter it again.

*NOTE: now the MIL aliases will work on your machine. You can type* `mil` *to take you to the root of the repo anytime*


### Starting a tmux session in the development container
When using ROS and Gazebo, many terminals are required. [tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) is a program which allows one terminal to be split into many terminals each called a panel. We highly recommend its use when runnning code, esspecially when working in a docker container.

1. Start a tmux session from the container (allows you to have multiple terminals within the container) `tmux new`
1. Split the tmux session horizontally with `Control+B` then `"`.
1. Split the tmux session vertically with `Control+B` then `%`.
1. You can switch between the terminals with `Control+B` then up arrow / down arrow

## Build the repository
Now that you have everything set up, try compiling the code. The user_install script should have done this already, but this is how you will compile in the future.

We have a convenient alias for this, run `cm`

If something goes wrong, try the suggestions in [Getting Help](help)

## (Optional) Install proprietary software
MIL uses a few proprietary packages which cannot be distributed with the repo.
You are able to compile the repo and run simulations without them, but
they must be installed on the actual robots.

* `./scripts/install_bvtsdk` - installs the BlueView Imaging Sonar SDK
* `./scripts/install_flycap`- installs the Pointgrey Flycapture SDK

These scripts will prompt you for an encryption password. Ask a MIL leader for this.

## (Optional) Install UDEV rules
If you plan on running certain MIL sensors from your local machine
or are setting up the install on a robot, you must give your user
access to certain USB devices through UDEV.

`./scripts/install_udev_rules`

## Run some code
Now that the repository is built, try running something!

For example, [SubjuGator](/docs/subjugator/index)
