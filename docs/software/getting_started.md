# Getting Started

This page describes the recommended process of setting up a new Ubuntu system
to work with the MIL repository and its dependencies. Ubuntu is an operating system
(similar to OS X or Windows) that MIL (and other robotics labs) choose to use
because of its familiarity with ROS.

## System Requirements

To use Ubuntu in MIL, you have two options:

1. Use a **virtual machine**. A virtual machine uses your current operating system
to virtually host another operating system (in this case, Ubuntu). This will
likely cause your Ubuntu system to run slow (in some cases, it might be unusable)
and may cause you to experience issues with certain programs.

2. **Dual-boot your computer.** Dual booting will allocate a piece of your computer's
storage to a new operating system, which will be installed on your computer directly.
This dual-booted solution will not run on top of your current operating system, but
will instead run by directly using your computer's resources.

Dual-booting your computer is highly recommended over using a virtual machine.
However, if you **have an M-series (ARM) Mac computer**, you will need to use a 
virtual machine. Intel-based Macs may also experience some issues with dual-booting
Ubuntu.

### Hardware

Autonomous robotics is computationally expensive, especially when running simulations.

If you can, use a powerful computer. We recommend you have at least 8GB RAM dedicated 
to your solution and a modern i5 or better CPU. Many tasks such as simulation and 
computer vision will run faster on a system with a GPU.

## Installing Ubuntu

To install Ubuntu, please see here:

* **Dual-booting a Windows computer**: Dual-booting on windows is the best option
to install Ubuntu on a Windows computer. To complete the process, [look here](https://help.ubuntu.com/community/WindowsDualBoot).

* **Using a virtual machine on Windows:** You could also use a virtual machine
software on Windows to run Ubuntu. These softwares include VirtualBox or VMWare.
Choose which virtualization software works best for you and find a tutorial on how
to install Ubuntu.

* **Using Parallels on MacOS**: Parallels is a favorited virtual machine software
available on MacOS. It is not free, although there is a student discount available.
To see how to install Ubuntu on MacOS with parallels, see [here](https://peterwitham.com/videos/how-to-install-ubuntu-20-04-lts-on-parallels-for-mac/).

* **Using UTM on a Mac**: UTM is a virtual machine software that is compatible with
MacOS. The software is free, unlike Parallels, but requires more setup by the user.
For instructions on installing Ubuntu with UTM, [see here](https://mac.getutm.app/gallery/ubuntu-20-04).

## Starting from a clean slate

First, we will want to check if there are any updates. Sometimes this is already 
done for you.

To do this, we use a command-line tool on Ubuntu named ``apt``. This tool allows
for the management of packages in Ubuntu.

(To access the Terminal on Ubuntu, click the 9 dots in the bottom left corner
and find the Terminal app. You can later install a different terminal emulator,
if you'd like.)

    $ sudo apt update
    $ sudo apt upgrade

## Install git

To contribute changes, you will need to have [git](https://www.git-scm.com) installed. 
This program will be used to track and upload any changes that you make to the repository.

    $ sudo apt install git

## Cloning the repository

You need to clone (download) a copy of the repository onto your computer so you 
can make changes.

First clone the upstream (MIL's fork) version of the repo.
It is recommended that you first create a catkin workspace
and clone it into the `src` or that workspace.

    $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

Then, clone the repository (the `--recurse-submodules` is essential, as that is 
what pulls the submodules, `-j8` is a speed optimization that downloads up to 8 
submodules at a time). Cloning the repository allows you to have a local copy 
of the code on your computer.

    $ git clone --recurse-submodules -j8 https://github.com/uf-mil/mil.git

Navigate to the root of the repository now. If you have followed the 
instructions exactly, you can get there through:

    $ cd ~/catkin_ws/src/mil

### Run the setup scripts

If you are running Ubuntu, and prefer to simply run code directly on your "host"
operating system without using Docker, you can run the following scripts from 
the root of the repository.

    $ ./scripts/system_install
    $ ./scripts/user_install
    $ exec bash # or exec zsh if you have set up zsh

Exit the terminal and enter it again.

To build our tools, we use a tool that ROS provides us named `catkin_make`. This 
searches through all of our packages and compiles them together. If you want to 
run this tool from anywhere in the directory, use `cm`.

    $ cm

If something goes wrong, try the suggestions in [Getting Help](help).

### Configuring Git

If you have not configured Git to use your name/email on your new Ubuntu setup, 
you can use the script below to set up your Git configuration! It will help you 
register your name and email with Git and authenticate so that you can push to 
the repository.

    $ ./scripts/store_git.sh

### Join the Github organization
In order to contribute code, you need to be in the [uf-mil Github organization](https://github.com/uf-mil). 
First create an acount if you do not have one, then ask someone in MIL / on Slack 
to invite you.

## Viewing simulation
Now that you've built the repository, check to make sure that the simulation works
on your computer. To do this, we will use a tool called `roslaunch`, which launches
up a simulation of our submarine.

For this, you will need multiple terminal windows! To create a new window in the
default terminal, use `Ctrl + Shift + t`.

In window one, write:

    $ roslaunch sub8_launch gazebo_launch

In window two, write:

    $ gazebogui

The first command launches the server-side version of Gazebo, while the second command
launches the Gazebo GUI - aka, the thing you will actually interact with! If all
goes according to plan, you should see our robot in its own little world!

## What's next?
If the `catkin_make` didn't fail on your computer, you're all good to go! 
Congratulations, and welcome to MIL!

The next step is to get assigned a task and to dive in. You should have the setup
to do so now!
