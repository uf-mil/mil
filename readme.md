# General Information

The [Machine Intelligence Lab (MIL)](http://mil.ufl.edu/) focuses on a few different projects throughout the year. These include the following vehicles:
* [SubjuGator](http://subjugator.org) - An Autonomous Underwater Vehicle (AUV) that is entered in the [RoboSub competition](http://www.robonation.org/competition/robosub)
* [PropaGator](http://propagator.org) - An Autonomous Surface Vehicle (ASV) that is entered in the [RoboBoat competition](http://www.robonation.org/competition/roboboat)
* [NaviGator](http://www.navigatoruf.org) - An Autonomous Surface Vehicle (ASV) that is entered in the [Maritime RobotX Challenge](https://www.robotx.org)

# Getting involved

The project that is currently active typically depends on which competition date is nearest. A fool proof way to check is to look at the activity on our Github organization, [uf-mil](https://github.com/uf-mil) or simply stop by our lab on the UF main campus in MAEC 126. Each project depends on the [software-common](https://github.com/uf-mil/software-common) repository, which is our core software stack. The [wiki](https://github.com/uf-mil/software-common/wiki) for that repository contains information that applies to all of our vehicles. The wiki's on each project contain vehicle-specific information. The team lead for each project is usually listed on the wiki home page, so feel free to email them for more information.

In order to develop software for the robot, you will need to become familiar with a few tools that are the building blocks of our codebase and workflow:
* [Ubuntu](http://www.ubuntu.com/) - This is the linux-based operating system that we use. Currently, only Ubuntu 16.04 is supported by our software stack, so that is the version you will need. If you are not familiar with Linux and Bash, I recommend you read through this [tutorial](http://ryanstutorials.net/linuxtutorial/).
* [Python](https://www.python.org/) - This is the main programming language used on the project and the one recommended to new programmers. C++ is also an option if you feel more comfortable with it, but the learning curve to use it with ROS is a bit steeper.
* [ROS](http://www.ros.org/) - The Robot Operating System is what we use to ensure that nodes running on the robot can communicate properly. Currently, some of our projects are still using ROS Indigo; however, we are migrating them to ROS Kinetic. If you are working on the active project, chances are you will be using Kinetic. For more information, we recommend visiting the site and going through a couple of the tutorials.
* [Git](https://git-scm.com/) - Git is our version control software; it allows us to keep track of the history of files so that we know who made which changes and revert back to previous versions if neccessary. We use a [forking workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow) to facilitate contributions. For a beginner's introduction to git, see this [tutorial](https://git-scm.com/doc).
* [OpenCV](http://opencv.org/) - This library enables us to perform complex robot vision tasks in a simple and elegant way. If you are interested in robot vision, this will be your go to.

# Setting Up the Development Environment

If you have decided to develop for one of our projects, then you will need to set up a development environment that is compatible with it. All of the projects have many dependencies, but there is a convenient install script in this repository to fetch and install them. The script covers all MIL projects that are currently on Ubuntu 16.04 and ROS Kinetic and will allow you to select which one(s) you would like to set up. If the default install location of the catkin workspace at `~/mil_ws` is not ideal, then a different path can be passed to the script with the -c option.

The following command will fetch and run the script:

    curl -s https://raw.githubusercontent.com/uf-mil/installer/master/install.sh | bash

The install script is intended to handle *every single thing* that needs to be installed to develop for the robot. If it does not work, something has gone terribly wrong and it needs to be fixed. If you resolve an issue while installing, please fix it in the install script and submit a pull-request with the changes. Otherwise, notify the script's maintainer (currently [Anthony Olive](mailto:anthony@iris-systems.net)).

The install script can accept arguments. For example, `./install.sh -c ~/my_awesome_ws` will generate a catkin workspace with the selected repositories in it at `~/my_awesome_ws` or use an existing workspace if one exists at that location. It will make all of the files and directories it needs within that workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be changed. This means that the script will respect workspaces with other repositories already present in them.
