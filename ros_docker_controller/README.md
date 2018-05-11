# ROS Docker Controller

## Why?
Docker allows for lightweight containerization using the base compatible OS. As such, it is possuble to run and develop the MIL software stack inside docker container, as long the base OS runs 64-bit linux. Meaning, if you are using a non-ubuntu linux system, or running Windows' Ubuntu-Shell, you may use docker to develop and run MIL software.

## How?
The included `Dockerfile` initializes an image with Ubuntu-xenial 16.04 OS with and ROS installed, then downloads the install script which you may then run once the container is opened. `ros_docker_controller.py` provides a curses interface for you to easily manage and control docker.

Running `ros_docker_controller.py` for the first time will initalize the image build process. After it is completed, it will open `RustyROS` container for the first time, asking for selection of an image (`mil_image:latest`). Then an options menu appears that provides interfaces to control and manage docker.

Docker containers hold their save-state even when stopped (unless they were to be pruned). Moreover, containers can be commited into images, allowing a more perminant save, and allowing the option to open a completely different container at some save-state.
