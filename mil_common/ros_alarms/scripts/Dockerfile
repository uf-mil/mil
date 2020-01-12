# This Docker file generates an image based on Ubuntu 16.04 and installs the
# packages required for use with Jenkins CI. It also installs all of the
# dependencies of ros_alarms.

FROM ubuntu:xenial

MAINTAINER Anthony Olive <anthony@iris-systems.net>

# Update the image and install tools needed to create the image
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install sudo \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install lsb-release \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install git \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install wget \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install python-flake8 \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install clang-format-3.8 \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y autoremove --purge \
	&& apt-get -y clean \
	&& rm -rf /var/lib/apt/lists/* \
	&& rm -f /var/cache/apt/*.bin

# Install all dependencies of txros, which is a dependency of ros_alarms
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list \
	&& echo "deb-src http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros.list \
	&& apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
	&& DEBIAN_FRONTEND=noninteractive apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install ros-kinetic-ros-base \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install ros-kinetic-roscpp-tutorials \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install ros-kinetic-tf \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install python-twisted \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y autoremove --purge \
	&& apt-get -y clean \
	&& rm -rf /var/lib/apt/lists/* \
	&& rm -f /var/cache/apt/*.bin

# Create a jenkins user for Jenkins CI and grant it passwordless sudo access
RUN useradd --uid 10000 --create-home --shell /bin/bash jenkins \
	&& echo "" >> /etc/sudoers \
	&& echo "jenkins ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Make Jenkins the default user for the image
USER jenkins
WORKDIR /home/jenkins
SHELL ["/bin/bash", "-c"]

# Create a volume to access the external files from Jenkins
RUN mkdir /home/jenkins/.jenkins
VOLUME /home/jenkins/.jenkins

# Create a catkin workspace for the package
RUN source /opt/ros/kinetic/setup.bash \
	&& mkdir -p ~/catkin_ws/src \
	&& cd ~/catkin_ws/src \
	&& catkin_init_workspace
