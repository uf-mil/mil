# Noetic Migration

Welcome! Thanks for helping with the migration to ROS Noetic. This guide will attempt to walk you through some of the tools we will use and some of the steps we will take to achieve ROS Noetic compatability.

## Overview

**So, what even is the problem here? What is "ROS Melodic" and "ROS Noetic", and why do we need to migrate between them?**

First, ROS is the Robot Operating System, which controls how our robots perform. Released in 2018, ROS Melodic is the version our code currently operates on. ROS Noetic was released in 2020 and is the next major version of ROS after ROS Melodic.

Like most software, ROS Noetic fixes a lot of bugs and adds helpful features that can be used when programming our robots. Therefore, many systems are migrating over to ROS Noetic, including the VRX competition, one of the main competitions we compete in. To ensure compability with ROS Noetic in the future, we want to start migrating earlier rather than later so that we aren't left behind and can use the latest-and-greatest features on our robots!

So the migration between ROS Melodic is easy-peasy, right? A few setup scripts here, a touch of code editing there? Well, not exactly. There are a few major components of our systems that must be updated to work with ROS Noetic. This includes:

* Updating all **Python 2** to be compatible with **Python 3**
* Updating ROS packages to use a different version of CMake
* Verifying dependency compatibility with ROS Noetic
* Updating some dependencies, such as OpenCV and PCL
* *[Other various changes...](http://wiki.ros.org/noetic/Migration)*

We will work through these changes piece by piece to make sure our systems are up to date.
