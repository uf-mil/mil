# This Docker file generates an image based on Ubuntu 16.04 and installs the
# packages required for use with Jenkins CI. It uses the install script to
# configure the image for building NaviGator.

FROM ubuntu:xenial

MAINTAINER Nicholas Suhlman <suhlman3@gmail.com>

# Configure the install script for NaviGator with CUDA
ENV DOCKER true
ENV INSTALL_NAV true
ENV INSTALL_CUDA true
ENV INSTALL_FLYCAP true

# Allow the user to pass in the SDK password with '--build-arg SDK_PASSWORD="password"'
ARG SDK_PASSWORD

# Clean cache
RUN DEBIAN_FRONTEND=noninteractive rm -rf /var/lib/apt/lists/* \
        && DEBIAN_FRONTEND=noninteractive apt-get clean

# Update the image and install tools needed to create the image
RUN DEBIAN_FRONTEND=noninteractive apt-get update --fix-missing\
	&& DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install sudo \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install curl \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install wget \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y install lightdm \
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

# Run the install script in Docker mode (with the DOCKER environment variable)
RUN wget https://raw.githubusercontent.com/uf-mil/installer/master/install.sh \
	&& chmod +x install.sh \
	&& bash install.sh \
	&& rm install.sh \
	&& sudo apt-get -y clean \
	&& sudo rm -rf /var/lib/apt/lists/* \
	&& sudo rm -f /var/cache/apt/*.bin
