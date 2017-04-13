# This Docker file generates an image based on Ubuntu 16.04 and installs the
# packages required for use with Jenkins CI. It uses the install script to
# configure the image for building SubjuGator.

FROM ubuntu:xenial

MAINTAINER Anthony Olive <anthony@iris-systems.net>

# Configure the install script for SubjuGator with CUDA
ENV DOCKER true
ENV INSTALL_SUB true
ENV INSTALL_CUDA true

# Allow the user to pass in the BlueView SDK with '--build-arg BVSDK_PASSWORD="password"'
ARG BVSDK_PASSWORD

# Update the image and install tools needed to create the image
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install sudo \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install curl \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install wget \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install bash-completion \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install openjdk-8-jdk \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y autoremove --purge \
	&& curl --create-dirs -sSLo /usr/share/jenkins/slave.jar https://repo.jenkins-ci.org/public/org/jenkins-ci/main/remoting/3.7/remoting-3.7.jar \
	&& apt-get -y clean \
	&& rm -rf /var/lib/apt/lists/* \
	&& rm -f /var/cache/apt/*.bin

# Create a jenkins user for Jenkins CI and grant it passwordless sudo access
RUN useradd --uid 10000 --create-home --shell /bin/bash --groups sudo jenkins \
	&& echo "" >> /etc/sudoers \
	&& echo "jenkins ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Make Jenkins the default user for the image
USER jenkins
WORKDIR /home/jenkins
SHELL ["/bin/bash", "-c"]

# Create a volume to access the external files from Jenkins
RUN mkdir /home/jenkins/.jenkins
VOLUME /home/jenkins/.jenkins

# Run the install script in Docker mode (with the DOCKER environment variable)
RUN wget https://raw.githubusercontent.com/uf-mil/installer/master/install.sh \
	&& chmod +x install.sh \
	&& bash install.sh \
	&& rm install.sh \
	&& sudo apt-get -y clean \
	&& sudo rm -rf /var/lib/apt/lists/* \
	&& sudo rm -f /var/cache/apt/*.bin
