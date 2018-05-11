FROM ubuntu:xenial

MAINTAINER Daniel Volya (RustyBamboo)

ENV TERM xterm-256color

# Install the basics
RUN apt-get update \
	&& apt-get -y dist-upgrade \
	&& apt-get -y install sudo \
	&& apt-get -y install curl \
	&& apt-get -y install wget \
    && apt-get -y install pciutils

# RUN apt-get -y install lightdm \
RUN apt-get -y autoremove --purge \
	&& apt-get -y clean \
	&& rm -rf /var/lib/apt/lists/* \
    && rm -f /var/cache/apt/*.bin

RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# Add and do user stuff
RUN useradd --create-home --shell /bin/bash --groups sudo --password ion mil \
    	&& echo "mil ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER mil
WORKDIR /home/mil
SHELL ["/bin/bash", "-c"]

RUN mkdir .ssh

# bootstrap rosdep
RUN sudo rosdep init \
    && rosdep update

RUN wget https://raw.githubusercontent.com/uf-mil/installer/master/install.sh \
	&& chmod +x install.sh

RUN  sudo apt-get -y clean \
	&& sudo rm -rf /var/lib/apt/lists/* \
    && sudo rm -f /var/cache/apt/*.bin

# Will be replaced, if selected, by ros_docker_controller script
# COPY ssh_github_key

CMD ["bash"]
