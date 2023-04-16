#!/bin/bash
set -euo pipefail

Res='\e[0;0m'
# shellcheck disable=SC2034
Red='\e[0;31m'
Gre='\e[0;32m'
Yel='\e[0;33m'
Pur='\e[0;35m'

color() {
	printf "%b" "$1"
}

# header display functions
hash_header() { echo "########################################"; }

# We only want to clear if the user is actually using a GUI terminal
if [[ -z ${TERM} ]]; then
	clear
fi

# Display header
cat <<EOF
$(color "$Pur")
         &@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#
       @@@@@@*,...................................................,/@@@@@@
     ,@@@@                                                            ,@@@@
     (@@@(                      /////&@* (@%////*                      &@@@*
     (@@@(                 #         %@* (@(         &                 &@@@*
     (@@@(                @@@@       %@* (@(      ,@@@@                &@@@*
     (@@@(               &@@@@@@%    %@* (@(    @@@@@@@#               &@@@*
     (@@@(              %/  @@@@@@@. %@* (@( ,@@@@@@&  #,              &@@@*
     (@@@(             *&     ,@@@@@@@@* (@@@@@@@@      @.             &@@@*
     (@@@(             @         (@@@@@* (@@@@@,         @             &@@@*
     (@@@(            @             @@@* (@@#            ,@            &@@@*
     (@@@(           &               %@* (@(              /#           &@@@*
     (@@@(          %/               %@* (@(               %,          &@@@*
     (@@@(         *@                %@* (@(                @.         &@@@*
     (@@@(         @                 %@* (@(                 @         &@@@*
     (@@@(        @                  %@* (@(                 ,@        &@@@*
     (@@@(  .@@@@@@@@            &@@@@@* (@@@@@#            @@@@@@@@   &@@@*
     (@@@#  .%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   &@@@*
     .@@@@.                                                           (@@@@
       @@@@@@%////////////////////////////////////////////////////(&@@@@@@
         #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/
$(color "$Gre")
$(hash_header)$(hash_header)
                                  Welcome to the
                             Machine Intelligence Lab
                                      at the
                               University of Florida
$(hash_header)$(hash_header)
$(color "$Yel")
This script will help to get your system installed with the needed dependencies.
You shouldn't need to do much - sit back and watch the magic happen before your
eyes!$(color "$Res")
EOF

sleep 2

mil_system_install() {
	sudo apt install -y "$@"
}

cat <<EOF
$(color "$Pur")
$(hash_header)
Fetching latest apt packages...
$(hash_header)
EOF

# Update apt
sudo apt update

# Installation for virtual machines
# Installs the apt-add-repository command
sudo apt-get install software-properties-common -y

# Installs keyboard config without prompting for input
sudo DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration -y # Weird bug

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Fetched latest apt packages.
$(color "$Pur")Installing needed Linux dependencies...
$(hash_header)$(color "$Res")

EOF

# System dependencies
mil_system_install apt-utils
mil_system_install --no-install-recommends \
	ca-certificates \
	curl \
	tzdata \
	dirmngr \
	gnupg2 \
	lsb-release \
	python3 \
	python3-pip \
	python2 \
	ruby \
	wget \
	vim \
	expect \
	doxygen \
	doxygen-doc \
	doxygen-gui \
	graphviz

# Attempt to install vcstool using apt-get or pip if apt-get does not work
sudo apt install -y python3-vcstool || sudo pip3 install -U vcstool

# Install Python 2 pip
sudo add-apt-repository universe
sudo apt update
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
python2 get-pip.py
rm get-pip.py

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Installed needed Linux dependencies.
$(color "$Pur")Setting up ROS distributions...
$(hash_header)$(color "$Res")

EOF

# ROS apt source
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Install Gazebo apt source
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Pull ROS apt key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Pull gazebo apt key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key D2486D2DD83DB69272AFE98867170598AF249743

# Pull Gazebo apt key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# Update apt again and install ros
sudo apt update

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Set up sources for ROS distributions.
$(color "$Pur")Downloading ROS Noetic...
$(hash_header)$(color "$Res")
EOF

mil_system_install ros-noetic-desktop-full
# Install additional dependencies not bundled by default with ros
# Please put each on a new line for readability
mil_system_install \
	ros-noetic-serial \
	ros-noetic-tf2-sensor-msgs \
	ros-noetic-geographic-msgs \
	ros-noetic-vision-msgs \
	ros-noetic-velodyne \
	ros-noetic-usb-cam \
	ros-noetic-joy \
	ros-noetic-spacenav-node \
	ros-noetic-velodyne-simulator \
	ros-noetic-hector-gazebo-plugins \
	ros-noetic-joy-teleop \
	ros-noetic-key-teleop \
	ros-noetic-robot-localization \
	ros-noetic-teleop-tools \
	ros-noetic-teleop-twist-keyboard \
	ros-noetic-ros-control \
	ros-noetic-ros-controllers \
	ros-noetic-tf2-tools

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Downloaded ROS Noetic.
$(color "$Pur")Installing Python dependencies...
$(hash_header)$(color "$Res")
EOF

# Documentation dependencies
mil_system_install python3-pip python3-setuptools

# Disable "automatic updates" Ubuntu prompt (thanks to https://askubuntu.com/a/610623!)
sudo sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades

# Install Python 3 dependencies
sudo pip3 install -r requirements.txt

# Link a python executable to Python 2 for temporary backwards compatibility
if which python >/dev/null 2>&1; then
	echo "Python executable already exists."
else
	echo "Attempting to symlink python2 to python..."
	sudo ln -s /usr/bin/python2 /usr/bin/python
	echo "Symlinked Python executable! Nice!"
fi

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Downloaded and setup Python dependencies.
$(color "$Pur")Initializing rosdep...
$(hash_header)$(color "$Res")
EOF

# Initialize rosdep
sudo apt-get install python3-rosdep

# Update rosdep
sudo rm -rf /etc/ros/rosdep/sources.list.d/* # Delete this file first - if not deleted, error could be thrown
sudo rosdep init
sudo rosdep update

# It is generally assumed that users will use ~/catkin_ws, but
# setting CATKIN_WS prior to running this script will change this behavior.
if [[ -z ${CATKIN_DIR:-""} ]]; then
	CATKIN_DIR="$HOME/catkin_ws"
	echo "Using default catkin workspace $CATKIN_DIR"
else
	echo "Using custom catkin workspace $CATKIN_DIR"
fi
CATKIN_SOURCE_DIR="$CATKIN_DIR/src"
MIL_REPO_DIR="$CATKIN_SOURCE_DIR/mil"

# Clone repository
mil_user_install_dependencies() {
	sudo apt update
	sudo apt install -y \
		git \
		tmux \
		vim \
		htop \
		tmuxinator \
		awscli \
		net-tools \
		cifs-utils \
		nmap \
		fd-find \
		ripgrep \
		lsd \
		fzf
}

# Add line to user's bashrc which source the repo's setup files
# This allows us to update aliases, environment variables, etc
mil_user_setup_rc() {
	# Line(s) added to ~/.bashrc or ~/.zshrc
	# Note that for backwards compatibility this should not be changed
	# unless you have a very good reason.
	BASH_RC_LINES=". $MIL_REPO_DIR/scripts/setup.bash"
	if [[ $SHELL == "/usr/bin/zsh" ]]; then
		# User is using zsh
		if grep -Fq "$BASH_RC_LINES" ~/.zshrc; then
			echo "milrc is already sourced in ~/.zshrc, skipping"
		else
			echo "Adding source of milrc to ~/.zshrc"
			{
				echo ""
				echo "# Setup environment for MIL development"
				echo "$BASH_RC_LINES"
			} >>~/.zshrc
		fi
	else
		# User is using zsh
		if grep -Fq "$BASH_RC_LINES" ~/.bashrc; then
			echo "milrc is already sourced in ~/.bashrc, skipping"
		else
			echo "Adding source of milrc to ~/.bashrc"
			{
				echo ""
				echo "# Setup environment for MIL development"
				echo "$BASH_RC_LINES"
			} >>~/.bashrc
		fi
	fi
}

# Sets up the catkin workspace so that user can build
# If the repo is already cloned here, it will build the MIL code
# catkin_init_workspace is superfluous, catkin_make is all you need
mil_user_setup_init_catkin() {
	mkdir -p "$CATKIN_SOURCE_DIR"
	catkin_make -C "$CATKIN_DIR"
}

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Initialized rosdep.
$(color "$Pur")Installing user tools and shell...
$(hash_header)$(color "$Res")
EOF

mil_user_install_dependencies
mil_user_setup_rc
set +u
. /opt/ros/noetic/setup.bash
set -u

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Setup user tools and shell.
$(color "$Pur")Compiling repository...
$(hash_header)$(color "$Res")
EOF

touch ~/tmux.conf
if grep 'set -g default-terminal "screen-256color"' ~/tmux.conf; then
	echo "Tmux already has 256 color support, skip this step"
else
	echo 'set -g default-terminal "screen-256color"' >>~/.tmux.conf
fi

mil_user_setup_init_catkin
