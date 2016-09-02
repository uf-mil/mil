#!/bin/bash

NOCOLOR='\033[0m'
LOGCOLOR='\033[1;36m'
PASSCOLOR='\033[1;32m'
WARNCOLOR='\033[1;31m'

LOGPREFIX="${LOGCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"
PASSTEXT="${PASSCOLOR}PASS"
FAILTEXT="${WARNCOLOR}FAIL"

instlog() {
	printf "$LOGPREFIX $@ $NOCOLOR\n"
}

instwarn() {
	printf "$WARNPREFIX $@ $NOCOLOR\n"
}


instpass() {
	printf "$PASSTEXT $NOCOLOR"
}


instfail() {
	printf "$FAILTEXT $NOCOLOR"
}

ros_git_get() {
	# Uasge example: ros_git_get git@github.com:jpanikulam/ROS-Boat.git
	NEEDS_INSTALL=true;
	INSTALL_URL=$1;
	builtin cd $CATKIN_DIR/src

	# Check if it already exists
	for folder in $CATKIN_DIR/src/*; do
		if ! [ -d $folder ]; then
			continue;
		fi

		builtin cd $folder
		if ! [ -d .git ]; then
			continue;
		fi
		LOCAL_BRANCH=`git name-rev --name-only HEAD`
		TRACKING_BRANCH=`git config branch.$LOCAL_BRANCH.merge`
		TRACKING_REMOTE=`git config branch.$LOCAL_BRANCH.remote`

		# Automatically checks if HTTPS is available
		REMOTE_URL=`git config remote.$TRACKING_REMOTE.url`
		if python -c "import re; _, have_url = re.split('https://github.com|git@github.com:', '$REMOTE_URL');_, want_url = re.split('https://github.com|git@github.com:', '$INSTALL_URL'); exit(have_url != want_url)"; then
			instlog "Already have package at url $INSTALL_URL"
			NEEDS_INSTALL=false;
			break;
		fi
		builtin cd $CATKIN_DIR/src
	done
	if $NEEDS_INSTALL; then
		instlog "Installing $INSTALL_URL in $CATKIN_DIR/src"
		git clone -q $INSTALL_URL --depth=1
	fi
}


#======================#
# Script Configuration #
#======================#

# Sane installation defaults for no argument cases
REQUIRED_OS="trusty"
CATKIN_DIR=~/mil_ws
INSTALL_ALL=false
INSTALL_SUB=false
INSTALL_NAV=false

# Retrievs information about the location of the script
SCRIPT_PATH="`readlink -f ${BASH_SOURCE[0]}`"
SCRIPT_DIR="`dirname $SCRIPT_PATH`"

# Convert script arguments to variables
while [ "$#" -gt 0 ]; do
	case $1 in
		-h) printf "\nUsage: $0\n"
			printf "\n    [-c] catkin_workspace (Recommend: ~/mil_ws)\n"
			printf "\n    [-a] Installs everything needed for all MIL projects\n"
			printf "\n    [-s] Installs everything needed for SubjuGator 8\n"
			printf "\n    [-n] Installs everything needed for Navigator\n"
			printf "\n    example: ./install.sh -c ~/mil_ws\n"
			printf "\n"
			exit 0
			;;
		-c) CATKIN_DIR="$2"
			shift 2
			;;
		-a) INSTALL_ALL=true
			shift 1
			;;
		-s) INSTALL_SUB=true
			shift 1
			;;
		-n) INSTALL_NAV=true
			shift 1
			;;
		-?) instwarn "Option $1 is not implemented"
			exit 1
			;;
	esac
done

if !($INSTALL_ALL || $INSTALL_SUB || $INSTALL_NAV); then
    echo "A MIL project must be selected for install"
    echo "Run ./install.sh -h for more information"
    exit 1
elif ($INSTALL_ALL); then
    INSTALL_SUB=true
    INSTALL_NAV=true
fi


#==================#
# Pre-Flight Check #
#==================#

instlog "Starting the pre-flight system check to ensure installation was done properly"

# The lsb-release package is critical to check the OS version
# It may not be on bare-bones systems, so it is installed here if necessary
sudo apt-get update -qq
sudo apt-get install -qq lsb-release

# Ensure correct OS is installed
DTETCTED_OS="`lsb_release -sc`"
if [ $DTETCTED_OS = $REQUIRED_OS ]; then
	OS_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	OS_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
	echo "OS distribution and version check"

if !($OS_CHECK); then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
	instwarn "Navigator requires Ubuntu 14.04 (trusty)"
	exit 1
fi


#=========================================#
# Repository and Common Dependancy Set Up #
#=========================================#

# Make sure script dependencies are installed on bare bones installations
instlog "Installing install script dependencies"
sudo apt-get install -qq wget aptitude fakeroot ssh git

# Add software repositories for ROS and Gazebo
instlog "Adding ROS and Gazebo PPAs to software sources"
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu trusty main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main\" > /etc/apt/sources.list.d/gazebo-latest.list"

# Get the GPG signing keys for the above repositories
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Add software repository for Git-LFS
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

# Install ROS and other project dependencies
instlog "Installing ROS and Gazebo"
sudo apt-get update -qq
sudo apt-get install -qq python-catkin-pkg python-rosdep
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
    sudo apt-get install -qq ros-indigo-desktop
else
    sudo apt-get install -qq ros-indigo-desktop-full
fi

# Break the ROS Indigo metapackage and install an updated version of Gazebo
sudo aptitude unmarkauto -q '?reverse-depends(ros-indigo-desktop-full) | ?reverse-recommends(ros-indigo-desktop-full)'
sudo apt-get purge -qq ros-indigo-gazebo*
sudo apt-get install -qq gazebo7
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control

# Source ROS configurations for bash on this user account
source /opt/ros/indigo/setup.bash
if !(cat ~/.bashrc | grep --quiet "source /opt/ros"); then
	echo "" >> ~/.bashrc
	echo "# Sets up the shell environment for ROS" >> ~/.bashrc
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update

instlog "Installing common dependencies from the Ubuntu repositories"

# Utilities for building and package management
sudo apt-get install -qq cmake binutils-dev python-pip

# Common backend libraries
sudo apt-get install -qq libboost-all-dev
sudo apt-get install -qq python-dev python-scipy python-numpy python-serial

# Point clouds
sudo apt-get install -qq libpcl-1.7-all libpcl-1.7-all-dev

# Motion planning
sudo apt-get install -qq libompl-dev

# Visualization and graphical interfaces
sudo apt-get install -qq libvtk5-dev python-vtk
sudo apt-get install -qq python-qt4-dev python-qt4-gl
sudo apt-get install -qq python-opengl freeglut3-dev libassimp-dev

# Tools
sudo apt-get install -qq git-lfs
git lfs install
sudo apt-get install -qq tmux

# Libraries needed by txros
sudo apt-get install -qq python-twisted socat

# Libraries needed by the old simulator
sudo apt-get install -qq python-pygame

instlog "Installing common ROS dependencies"

# Hardware drivers
sudo apt-get install -qq ros-indigo-driver-base

# Cameras
sudo apt-get install -qq ros-indigo-camera-info-manager
sudo apt-get install -qq ros-indigo-camera1394
sudo apt-get install -qq ros-indigo-stereo-image-proc

# Image compression
sudo apt-get install -qq ros-indigo-rosbag-image-compressor ros-indigo-compressed-image-transport ros-indigo-compressed-depth-image-transport

# Point clouds
sudo apt-get install -qq ros-indigo-pcl-ros ros-indigo-pcl-conversions

# Lie Groups using Eigen
sudo apt-get install -qq ros-indigo-sophus

# Controller
sudo apt-get install -qq ros-indigo-control-toolbox ros-indigo-controller-manager
sudo apt-get install -qq ros-indigo-hardware-interface ros-indigo-transmission-interface ros-indigo-joint-limits-interface

instlog "Installing common dependencies from Python PIP"

# Package management
sudo pip install -q -U setuptools

# Service identity verification
sudo pip install -q -U service_identity

# Utilities
sudo pip install -q -U argcomplete
sudo pip install -q -U tqdm
sudo pip install -q -U pyasn1
sudo pip install -q -U characteristic

# Machine Learning
sudo pip install -q -U scikit-learn > /dev/null 2>&1

# Visualization
sudo pip install -q -U mayavi > /dev/null 2>&1

instlog "Cloning common Git repositories we need to build"
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/ros-simulation/gazebo_ros_pkgs.git


#===================================#
# Navigator Dependency Installation #
#===================================#

if ($INSTALL_NAV); then
    instlog "Installing Navigator ROS dependencies"

    # Serial communications
    sudo apt-get install -qq ros-indigo-rosserial ros-indigo-rosserial-python ros-indigo-rosserial-arduino

    # Thruster driver
    sudo apt-get install -qq ros-indigo-roboteq-driver

    instlog "Installing Navigator dependencies from source"

    # Open Dynamics Engine
    rm -rf /tmp/pyode-build
    mkdir -p /tmp/pyode-build
    cd /tmp/pyode-build
    sudo apt-get build-dep -qq python-pyode
    sudo apt-get remove -qq python-pyode
    apt-get source --compile -qq python-pyode
    sudo dpkg -i python-pyode_*.deb

    # Navigator currently depends on the Sub8 repository
    # This may change soon, but this will install Sub8 for now
    INSTALL_SUB=true
fi


#==============================#
# Sub8 Dependency Installation #
#==============================#

if ($INSTALL_SUB); then
    instlog "Installing Sub8 dependencies from the Ubuntu repositories"

    # Optical character recognition
    sudo apt-get install -qq tesseract-ocr

    # Hardware drivers
    sudo apt-get install -qq libusb-1.0-0-dev

    instlog "Installing Sub8 ROS dependencies"

    # 3D Mouse
    sudo apt-get install -qq ros-indigo-spacenav-node

    instlog "Installing Sub8 dependencies from Python PIP"

    # Libraries needed by the hydrophone board
    sudo pip install -q -U crc16
fi


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Set up catkin workspace directory
if !([ -f $CATKIN_DIR/src/CMakeLists.txt ]); then
    instlog "Generating catkin workspace at $CATKIN_DIR"
    mkdir -p "$CATKIN_DIR/src"
    cd "$CATKIN_DIR/src"
    catkin_init_workspace
    catkin_make -C "$CATKIN_DIR"
else
    instlog "Using existing catkin workspace at $CATKIN_DIR"
fi




ros_git_get https://github.com/uf-mil/Sub8.git




# Move the cloned git repository to the catkin workspace in semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
    if [ -d ~/Navigator ]; then
        mv ~/Navigator "$CATKIN_DIR/src"
    elif [ -d ~/Sub8 ]; then
        mv ~/Navigator "$CATKIN_DIR/src"
    fi
fi

# Source the workspace's configurations for bash on this user account
source "$CATKIN_DIR/devel/setup.bash"
if !(cat ~/.bashrc | grep --quiet "source $CATKIN_DIR/devel/setup.bash"); then
	echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
fi

# Check if the Navigator repository is present; if it isn't, download it
if ($INSTALL_NAV) && !(ls "$CATKIN_DIR/src" | grep --quiet "Navigator"); then
    instlog "Downloading up the Navigator repository"
    cd $CATKIN_DIR/src
    git clone -q https://github.com/uf-mil/Navigator.git
    cd $CATKIN_DIR/src/Navigator
    git remote rename origin upstream
    instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
fi

# Check if the Sub8 repository is present; if it isn't, download it
if ($INSTALL_SUB) && !(ls "$CATKIN_DIR/src" | grep --quiet "Sub8"); then
    instlog "Downloading up the Sub8 repository"
    cd $CATKIN_DIR/src
    git clone -q https://github.com/uf-mil/Sub8.git
    cd $CATKIN_DIR/src/Sub8
    git remote rename origin upstream
    instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
fi


#==========================#
# Finalization an Clean Up #
#==========================#

# Attempt to build the Navigator stack on client machines
if !(env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
    instlog "Building Navigator's software stack with catkin_make"
    catkin_make -C "$CATKIN_DIR" -j8
fi

# Remove the initial install script if it was not in the Navigator repository
if !(echo "$SCRIPT_DIR" | grep --quiet "src/Navigator"); then
	rm -f "$SCRIPT_PATH"
fi
