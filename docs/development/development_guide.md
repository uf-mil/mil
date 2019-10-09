# Development Guide
This page describes the recommended process of building and running code in this repository.

Note, this guide assumes you have already cloned the repository as described in the [Contributing Guide](contributing)

## Install Docker
It is recommended that you build and run MIL code within a Docker container.
This process will differ depending on your host operating system, but it
well documented for Windows, OSX, and several Linux operating systems.

* [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Windows](https://docs.docker.com/docker-for-windows/install/)
* [OSX](https://docs.docker.com/docker-for-mac/install/)

*Note: if you use Docker Desktop (Windows or OSX), be sure to move the RAM/CPU sliders to max*


### Alternative: run the setup scripts on your host operating system
If you are running Ubuntu 18.04 and prefer to simply run code directly on your "host"
operating sytem without using Docker, you can run the following scripts from the root of the repository

`./docker/system/system_install`

`./docker/dev/user_install`

## Build the containers
Now that you have docker installed, you must build the development container which
you will build and test code in.

From the root of the cloned repository in your *host operarating system*, run:

`./scripts/build_docker_containers`

This process requires internet and may take a while (up to an hour is common).

##  Run the developement container

Now that the developement container is built, you can run the container.

`./scripts/run_developement_container`

This will open a shell into a Ubuntu instance where you can build and test the
code in this repository (including your changes!).

Note that the clone of the repository from your host system is mounted into the container,
so changes to the code in your host system are also changed in the container and visa-versa.
This allows you to modify your code in your editor of choice on your host system while
building and testing changes within the container.

## Build the repository
Now that you are inside the development container, try building the repository.

We have a convienent alias for this, run `cm`

## Run some code
Now that the repository is built, try running something!
`roslaunch navigator_launch simulation.launch`
