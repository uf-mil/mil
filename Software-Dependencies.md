#**Software Dependencies**

**Please install all the dependencies below if you want to use the software on NaviGator. This page relies on you having done steps 1-4 in the [Getting Started](https://github.com/uf-mil/Navigator/wiki/Getting-started).**

####Please add any outside dependencies for programs you create to this page! Please please!

##**Open Motion Planning Library**

**The Open Motion Planning Library consists of many state-of-the-art sampling-based motion planning algorithms. It is used for path planning on the WAM-V**

To install from source follow the instructions at the link below word for word

    http://ompl.kavrakilab.org/installation.html

To install dependencies necessary to run OMPL code run:

    sudo apt-get install libompl-dev


##**Roboteq Motor Driver**

**The Roboteq controller comes complete with ROS software. Below is the information on the hardware as well as the command to download the software to run it on your computer.**

[View the motor controller here](http://www.roboteq.com/index.php/roboteq-products-and-services/brushless-dc-motor-controllers/mbl1660-detail)

    sudo apt-get install ros-indigo-roboteq-driver

##**Pygame**

**Pygame is a python GUI tool made for video game production. We use it to create simple visualizations for Navigator.**


    sudo apt-get install python-pygame

##**PyODE**

**PyODE is a set of open-source Python bindings for The Open Dynamics Engine, an open-source physics engine. PyODE also includes an XODE parser.**

**It is used for the navigator simulation and is necessary to run any simulation with graphics.**

PyODE seems to be broken, so we must compile from source. Thanks to Forrest for this discovery.

    rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb