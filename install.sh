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

check_host() {
	HOST="$1"

	# Attempts to ping the host to make sure it is reachable
	HOST_PING=$(ping -c 2 $HOST 2>&1 | grep "% packet" | awk -F'[%]' '{print $1}' | awk -F'[ ]' '{print $NF}')
	if [[ ! -z "$HOST_PING" ]]; then

		# Uses packet loss percentage to determine if the connection is strong
		if (( $HOST_PING < 25 )); then

			# Will return true if ping was successful and packet loss was below 25%
			echo "true"
		fi
	fi
}


#=======================#
# Configurable Defaults #
#=======================#

CATKIN_DIR=~/mil_ws
MIL_CONFIG_DIR=~/.mil
BASHRC_FILE=~/.bashrc
MILRC_FILE=$MIL_CONFIG_DIR/milrc
BVTSDK_DIR=$MIL_CONFIG_DIR/bvtsdk


#======================#
# Script Configuration #
#======================#

# Prompt the user to configure the installation if the script is not being used in Docker
if [[ "$DOCKER" != "true" ]]; then

	# Install no project by default, the user must select one
	SELECTED="false"
	INSTALL_SUB="false"
	INSTALL_PRO="false"
	INSTALL_NAV="false"

	# Set sane defaults for other install parameters
	SDK_PASSWORD=""
	ENABLE_USB_CAM="false"
	INSTALL_CUDA="false"
	INSTALL_BVTSDK="false"
	INSTALL_FLYCAP="false"

	# Prompt the user to enter a catkin workspace to use
	echo "Catkin is the ROS build system and it combines CMake macros and Python scripts."
	echo "The catkin workspace is the directory where all source and build files for the"
	echo "project are stored. Our default is in brackets below, press enter to use it."
	echo -n "What catkin workspace should be used? [$CATKIN_DIR]: " && read RESPONSE
	if [[ ! -z "$RESPONSE" ]]; then
		CATKIN_DIR=${RESPONSE/\~//home/$USER}
	fi
	echo ""

	if [[ ! -d $CATKIN_DIR/src/mil_common ]]; then
		echo "We use a forking workflow to facilitate code contributions on Github. This means"
		echo "that each user forks the main repository and has their own copy. In the"
		echo "repositories that we clone for projects, the main repository will be the"
		echo "'upstream' remote and your local fork will be the 'origin' remote. You should"
		echo "specify a fork URI for each repository you plan to push code to; otherwise,"
		echo "leave the field blank. These can also be set manually using this command:"
		echo "git remote add <remote_name> <user_fork_url>"
		echo -n "User fork URI for the mil_common repository: " && read SWC_USER_FORK
		echo ""
	fi

	# Prompt the user to select a project to install
	while [[ "$SELECTED" != "true" ]]; do
		echo "A MIL project must be selected for install"
		echo "	1. SubjuGator"
		echo "	2. PropaGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo "	3. NaviGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo -n "Project selection: " && read RESPONSE
		echo ""
		case "$RESPONSE" in
			"1")
				if [[ ! -d $CATKIN_DIR/src/SubjuGator ]]; then
					echo -n "User fork URI for the SubjuGator repository: " && read SUB_USER_FORK
				fi
				INSTALL_SUB="true"
				SELECTED="true"
				echo ""
			;;
			"2")
				echo "The PropaGator project has not been worked on since the dark ages of MIL, so it"
				echo "is not supported by this script."
				echo ""
			;;
			"3")
				echo "The NaviGator project was developed on Ubuntu 14.04 with ROS Indigo. Several"
				echo "dependencies no longer exist in ROS Kinetic, so in order to install it, ROS"
				echo "Indigo, the SubjuGator repository at an earlier date,  and all of the old"
				echo "SubjuGator dependencies will need to be downloaded and installed."
				echo -n "Do you still wish to proceed? [y/N] " && read RESPONSE
				if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
					if [[ ! -d $CATKIN_DIR/src/NaviGator ]]; then
						echo -n "User fork URI for the NaviGator repository: " && read NAV_USER_FORK
					fi
					INSTALL_NAV="true"
					SELECTED="true"
				fi
				echo ""
			;;
			"")
				echo "You must select one of the projects by entering it's number on the list"
				echo ""
			;;
			*)
				echo "$RESPONSE is not a valid selection"
				echo ""
			;;
		esac
	done

	# Prompt the user to install the BlueView SDK if it is not already installed
	if [[ ! -d $BVTSDK_DIR ]]; then
		echo "The BlueView SDK used to interface with the Teledyne imaging sonar is encrypted"
		echo "in order to protect the intellectual property of BlueView. If you will be doing"
		echo "work with the imaging sonar on your machine, it is recommended that you install"
		echo "this now. If not, you probably do not need to."
		echo -n "Do you wish to install the SDK? [y/N] " && read RESPONSE
		echo ""

		if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
			INSTALL_BVTSDK="true"
		fi
	fi

	# Detect whether or not the Flycapture SDK has already been installed
	if [[ -z "$(dpkg-query -W -f='${Status}' flycap 2>/dev/null | grep 'ok installed')" ]]; then
		echo "MIL projects use Point Grey machine vision cameras for perception. A user only"
		echo "needs to install the Flycapture SDK if they intend to connect one directly to"
		echo "their machine, which is unlikely. This SDK containes closed source binaries."
		echo -n "Do you wish to install the SDK? [y/N] " && read RESPONSE
		echo ""

		if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
			INSTALL_FLYCAP="true"
		fi
	fi

	# If the user chooses to install an SDK, retrieve the password from them
	if [[ "$INSTALL_BVTSDK" == "true" || "$INSTALL_FLYCAP" == "true" ]]; then
		echo "One or more of the selected SDKs is encrypted with a password. You need to"
		echo "obtain this password from one of the senior members of MIL."
		echo -n "Encryption password: " && read -s SDK_PASSWORD
		echo ""
		echo ""
	fi

	# Warn users about the security risks associated with enabling USB cameras before doing it
	if [[ ! -f /etc/udev/rules.d/40-pgr.rules && "$INSTALL_FLYCAP" == "true" ]]; then
		echo "MIL projects use Point Grey machine vision cameras for perception. A user only"
		echo "needs to enable access to USB cameras if they intend to connect one directly"
		echo "to their machine, which is unlikely. In order for a user to access a USB"
		echo "camera, a udev rule needs to be added that gives a group access to the hardware"
		echo "device on the camera. Long story short, this creates a fairly significant"
		echo "security hole on the machine that goes beyond the OS to actual device firmware."
		echo -n "Do you want to enable access to USB cameras? [y/N] " && read RESPONSE
		if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
			ENABLE_USB_CAM="true"
		fi
		echo ""
	fi

	# Detect whether or not a CUDA toolkit has already been installed
	if [[ ! -z "$(dpkg-query -W -f='${Status}' cuda 2>/dev/null | grep 'ok installed')" ||
	      ! -z "$(dpkg-query -W -f='${Status}' nvidia-cuda-toolkit 2>/dev/null | grep 'ok installed')" ]]; then
		INSTALL_CUDA="true"

	# Give users the option to install the CUDA toolkit if an Nvidia card is detected
	elif [[ "$INSTALL_NAV" != "true" && ! -z "$(lspci | grep 'VGA compatible controller: NVIDIA')" ]]; then
		echo "An Nvidia graphics card was detected on this machine. Some of the perception"
		echo "packages may run faster with or require CUDA parallel processing on the GPU."
		echo "CUDA is not required, but the script can install it automatically."
		echo "	* Ubuntu 16.04 supports the 7.5 version natively; however, the 8.0"
		echo "	  version can be installed through a repository maintained by Nvidia"
		echo "	  and runs much smoother. Since the 8.0 version seems stable, this"
		echo "	  is the version that the script will install on Ubuntu."
		echo "	  The 7.5 version can still be installed manually with the following"
		echo "	  command: sudo apt-get install nvidia-cuda-toolkit"
		echo "	* Debian 8.7 supports the 6.0 version natively. Since it is not"
		echo "	  officially supported by the Nvidia development team, installing"
		echo "	  the 7.5 or 8.0 version should be done by the user at their own"
		echo "	  risk. The 6.0 version is what will be installed on Debian."
		echo -n "Do you want to install CUDA Toolkit? [y/N] " && read RESPONSE
		if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
			INSTALL_CUDA="true"
		fi
		echo ""
	fi
fi


#==================#
# Pre-Flight Check #
#==================#

instlog "Acquiring root privileges"

# Gets the user to enter their password here instead of in the middle of the pre-flight check
# Purely here for aesthetics and to satisfy OCD
sudo true

# Make sure script dependencies are installed quietly on bare bones installations
sudo apt-get update -qq
sudo apt-get install -qq lsb-release
sudo apt-get install -qq iputils-ping
sudo apt-get install -qq curl
sudo apt-get install -qq wget
sudo apt-get install -qq git
sudo apt-get install -qq python-pip

instlog "Starting the pre-flight system check to ensure installation was done properly"

# Check whether or not github.com is reachable
# This also makes sure that the user is connected to the internet
if [[ "$(check_host github.com)" == "true" ]]; then
	NET_CHECK="true"
	echo -n "[ " && instpass && echo -n "] "
else
	NET_CHECK="false"
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Internet connectivity check"

if [[ "$NET_CHECK" != "true" ]]; then

	# The script will not allow the user to install without internet
	instwarn "Terminating installation due to the lack of an internet connection"
	instwarn "The install script needs to be able to connect to Github and other sites"
	exit 1
fi

# Set the required OS based on inputs and installed distribution
if [[ "$INSTALL_NAV" == "true" ]]; then
	REQUIRED_OS_ID="Ubuntu"
	REQUIRED_OS_CODENAME="trusty"
	REQUIRED_OS_RELEASE="14.04"
	ROS_VERSION="indigo"
elif [[ "$(lsb_release -si)" == "Debian" ]]; then
	REQUIRED_OS_ID="Debian"
	REQUIRED_OS_CODENAME="jessie"
	REQUIRED_OS_RELEASE="8.7"
	ROS_VERSION="kinetic"
else
	REQUIRED_OS_ID="Ubuntu"
	REQUIRED_OS_CODENAME="xenial"
	REQUIRED_OS_RELEASE="16.04"
	ROS_VERSION="kinetic"
fi

# Ensure that the correct OS is installed
DETECTED_OS_CODENAME="$(lsb_release -sc)"
if [[ "$DETECTED_OS_CODENAME" == "$REQUIRED_OS_CODENAME" ]]; then
	OS_CHECK="true"
	echo -n "[ " && instpass && echo -n "] "
else
	OS_CHECK="false"
	echo -n "[ " && instfail && echo -n "] "
fi
echo "OS distribution and version check"

# Prevent the script from being run as root
if [[ "$USER" != "root" ]]; then
	ROOT_CHECK="true"
	echo -n "[ " && instpass && echo -n "] "
else
	ROOT_CHECK="false"
	echo -n "[ " && instfail && echo -n "] "
fi
echo "User permissions check"

# Ensure that no ROS version is being sourced in the user's bash runcom file
if [[ -z "$(cat $BASHRC_FILE | grep /opt/ros)" ]]; then
	BASHRC_CHECK="true"
	echo -n "[ " && instpass && echo -n "] "
else
	BASHRC_CHECK="false"
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Bash runcom file check"

if [[ "$OS_CHECK" != "true" ]]; then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DETECTED_OS_CODENAME)"
	instwarn "This project requires $REQUIRED_OS_RELEASE $REQUIRED_OS_RELEASE ($REQUIRED_OS_CODENAME)"
	exit 1
fi

if [[ "$ROOT_CHECK" != "true" ]]; then

	# The script will not allow the user to install as root
	instwarn "Terminating installation due to forbidden user account"
	instwarn "The install script should not be run as root"
	exit 1
fi

if [[ "$BASHRC_CHECK" != "true" ]]; then

	# The script will not allow the user to install if ROS is being sourced
	instwarn "Terminating installation due to $BASHRC_FILE sourcing a ROS version"
	instwarn "This should be handled through the MIL runcom file instead"
	instwarn "Removing lines that source ROS, workspaces, or project aliases is recommended"
	instwarn "However, removing only lines that source ROS will be enough to clear this error"
	exit 1
fi


#===================================================#
# Repository and Set Up and Main Stack Installation #
#===================================================#

# Add software repository for ROS to software sources
instlog "Adding the ROS PPA to software sources"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list'
sudo sh -c 'echo "deb-src http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Add software repository for Git-LFS to software sources
instlog "Adding the Git-LFS packagecloud repository to software sources"
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

# Add software repository for Gazebo to software sources if ROS Indigo is being installed
if [[ "$ROS_VERSION" == "indigo" ]]; then
	instlog "Adding the Gazebo PPA to software sources"
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo.list'
	sudo sh -c 'echo "deb-src http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/gazebo.list'
	wget -q http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
fi

# Add software repository for Terry Guo's ARM toolchain if NaviGator is being installed
if [[ "$INSTALL_NAV" == "true" ]]; then
	instlog "Adding Terry Guo's ARM toolchain PPA to software sources"
	sudo mkdir -p /etc/apt/preferences.d
	sudo sh -c "echo 'Package: *\nPin: origin "ppa.launchpad.net"\nPin-Priority: 999' > /etc/apt/preferences.d/arm"
	sudo sh -c 'echo "deb http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo sh -c 'echo "deb-src http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xA3421AFB
fi

# Add software repository for Nvidia to software sources if the CUDA option was selected
if [[ "$INSTALL_CUDA" == "true" && "$REQUIRED_OS_ID" == "Ubuntu" ]]; then
	instlog "Adding the Nvidia PPA to software sources"
	sudo sh -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
	wget -q -O - http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub | sudo apt-key add -
fi

# Install ROS and a few ROS dependencies
instlog "Installing ROS $(tr '[:lower:]' '[:upper:]' <<< ${ROS_VERSION:0:1})${ROS_VERSION:1}"
sudo apt-get update -qq
sudo apt-get install -qq ros-$ROS_VERSION-desktop-full

# If ROS Indigo is being installed, break the metapackage and install an updated version of Gazebo
if [[ "$ROS_VERSION" == "indigo" ]]; then
	instlog "Installing the latest version of Gazebo"
	sudo apt-get install -qq aptitude
	sudo aptitude unmarkauto -q '?reverse-depends(ros-indigo-desktop-full) | ?reverse-recommends(ros-indigo-desktop-full)'
	sudo apt-get purge -qq ros-indigo-gazebo*
	sudo apt-get install -qq gazebo7
	sudo apt-get install -qq ros-indigo-gazebo7-ros-pkgs
fi

# Get information about ROS versions
instlog "Initializing ROS"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update

# Source ROS configurations for bash
source /opt/ros/$ROS_VERSION/setup.bash


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Set up catkin workspace directory
if [[ ! -f $CATKIN_DIR/src/CMakeLists.txt ]]; then
	instlog "Generating catkin workspace at $CATKIN_DIR"
	mkdir -p $CATKIN_DIR/src
	cd $CATKIN_DIR/src
	catkin_init_workspace
else
	instlog "Using existing catkin workspace at $CATKIN_DIR"
fi

# Download the clang-format rules for the ROS C++ style guide to the workspace
if [[ ! -f $CATKIN_DIR/.clang-format ]]; then
	wget -O $CATKIN_DIR/.clang-format https://raw.githubusercontent.com/uf-mil/installer/master/.clang-format
fi

# Only clone the github repositories if the script is not being used by Docker
if [[ "$DOCKER" != "true" ]]; then

	# Download the mil_common repository if it has not already been downloaded
	if [[ ! -d $CATKIN_DIR/src/mil_common ]]; then
		instlog "Downloading the mil_common repository"
		cd $CATKIN_DIR/src
		git clone --recursive -q https://github.com/uf-mil/mil_common.git
		cd $CATKIN_DIR/src/mil_common
		git remote rename origin upstream
		if [[ ! -z "$SWC_USER_FORK" ]]; then
			git remote add origin "$SWC_USER_FORK"
		fi
	fi

	# Download the SubjuGator repository if it has not already been downloaded and was selected for installation
	if [[ "$INSTALL_SUB" == "true" && ! -d $CATKIN_DIR/src/SubjuGator ]]; then
		instlog "Downloading the SubjuGator repository"
		cd $CATKIN_DIR/src
		git clone --recursive -q https://github.com/uf-mil/SubjuGator.git
		cd $CATKIN_DIR/src/SubjuGator
		git remote rename origin upstream
		if [[ ! -z "$SUB_USER_FORK" ]]; then
			git remote add origin "$SUB_USER_FORK"
		fi
	fi

	# Download the NaviGator repository if it has not already been downloaded and was selected for installation
	if [[ "$INSTALL_NAV" == "true" ]]; then
		if [[ ! -d $CATKIN_DIR/src/SubjuGator ]]; then
			instlog "Downloading the SubjuGator repository"
			cd $CATKIN_DIR/src
			git clone --recursive -q https://github.com/uf-mil/SubjuGator.git
			cd $CATKIN_DIR/src/SubjuGator
			instlog "Rolling back the SubjuGator repository; do not pull the latest version!"
			git reset --hard 0089e68b9f48b96af9c3821f356e3a487841e87e
			git remote remove origin
		fi
		if [[ ! -d $CATKIN_DIR/src/NaviGator ]]; then
			instlog "Downloading the NaviGator repository"
			cd $CATKIN_DIR/src
			git clone --recursive -q https://github.com/uf-mil/NaviGator.git
			cd $CATKIN_DIR/src/NaviGator
			git remote rename origin upstream
			if [[ ! -z "$NAV_USER_FORK" ]]; then
				git remote add origin "$NAV_USER_FORK"
			fi
		fi
	fi
fi


#================================#
# Common Dependency Installation #
#================================#

if [[ "$ENABLE_USB_CAM" == "true" ]]; then
	instlog "Enabling USB cameras (a reboot is required for this to take full effect)"
	sudo usermod -a -G dialout "$USER"
	sudo groupadd --gid 999 pgrimaging
	sudo usermod -a -G pgrimaging "$USER"
	sudo wget -q https://raw.githubusercontent.com/uf-mil/installer/master/40-pgr.rules -O /etc/udev/rules.d/40-pgr.rules
	sudo service udev restart
fi

instlog "Installing common dependencies from the $REQUIRED_OS_ID repositories"

# CUDA toolkit
if [[ "$INSTALL_CUDA" == "true" ]]; then
	if [[ "$REQUIRED_OS_ID" == "Ubuntu" ]]; then
		sudo apt-get install -qq cuda
	else
		sudo apt-get install -qq nvidia-cuda-toolkit
	fi
fi

# Scientific and technical computing
sudo apt-get install -qq python-scipy

# System tools
sudo apt-get install -qq vim
sudo apt-get install -qq ipython
sudo apt-get install -qq tmux
sudo apt-get install -qq htop
sudo apt-get install -qq sshfs

# Git-LFS for models and other large files
sudo apt-get install -qq git-lfs
cd $CATKIN_DIR
git lfs install --skip-smudge

# Debugging utility
sudo apt-get install -qq gdb

# Formatting utilities
sudo apt-get install -qq clang-format-3.8
sudo apt-get install -qq python-flake8

# Networking
sudo apt-get install -qq python-twisted

# Machine Learning
sudo apt-get install -qq python-sklearn

# Visualization
sudo apt-get install -qq mayavi2

instlog "Installing common ROS dependencies"

# Hardware drivers
sudo apt-get install -qq ros-$ROS_VERSION-joy
sudo apt-get install -qq ros-$ROS_VERSION-serial

# Messages
sudo apt-get install -qq ros-$ROS_VERSION-tf2-sensor-msgs

instlog "Installing common dependencies from Python PIP"

# Communication tool for the hydrophone board
sudo pip install -q -U crc16

# Visualization
sudo pip install -q -U tqdm

# Decrypt and extract the BlueView SDK for the Teledyne imaging sonar
if [[ "$INSTALL_BVTSDK" == "true" ]]; then
	instlog "Decrypting and installing the BlueView SDK"
	mkdir -p $MIL_CONFIG_DIR
	curl -s https://raw.githubusercontent.com/uf-mil/installer/master/libbvtsdk.tar.gz.enc | \
	openssl enc -aes-256-cbc -d -pass file:<(echo -n $SDK_PASSWORD) | tar -xpzC $MIL_CONFIG_DIR

	# If the SDK does not decrypt correctly due to an incorrect password, fail the installation
	if [[ ! -d $BVTSDK_DIR ]]; then
		instwarn "Terminating installation due to incorrect password for the BlueView SDK"
		exit 1
	fi
fi

# Decrypt and extract the Flycapture SDK for the Point Grey cameras
if [[ "$INSTALL_FLYCAP" == "true" ]]; then
	instlog "Decrypting and installing the Flycapture SDK"
	curl -s https://raw.githubusercontent.com/uf-mil/installer/master/flycapture-2-2.11.3.121-amd64.tar.gz.enc | \
	openssl enc -aes-256-cbc -d -pass file:<(echo -n $SDK_PASSWORD) | tar -xpzC /tmp

	# If the SDK does not decrypt correctly due to an incorrect password, fail the installation
	if [[ ! -d /tmp/flycapture-2-2.11.3.121-amd64 ]]; then
		instwarn "Terminating installation due to incorrect password for the Flycapture SDK"
		exit 1
	fi

	# Install the (unlisted...) dependencies needed for the Flycapture SDK
	sudo apt-get install -qq libraw1394-11
	sudo apt-get install -qq libusb-1.0-0
	sudo apt-get install -qq libgtk2.0-0
	sudo apt-get install -qq libgtkmm-2.4-1v5
	sudo apt-get install -qq libglademm-2.4-1v5
	sudo apt-get install -qq libgtkmm-2.4-dev
	sudo apt-get install -qq libglademm-2.4-dev
	sudo apt-get install -qq libgtkglextmm-x11-1.2-dev

	# Install the Flycapture SDK based on the install script packaged with it
	cd /tmp/flycapture-2-2.11.3.121-amd64
	sudo dpkg -i libflycapture-2*
	sudo dpkg -i libflycapturegui-2*
	sudo dpkg -i libflycapture-c-2*
	sudo dpkg -i libflycapturegui-c-2*
	sudo dpkg -i libmultisync-2*
	sudo dpkg -i libmultisync-c-2*
	sudo dpkg -i flycap-2*
	sudo dpkg -i flycapture-doc-2*
	sudo dpkg -i updatorgui*
fi

# If this is not being run to create a Docker image, link the BlueView SDK to mil_blueview_driver
if [[ -d $BVTSDK_DIR && "$DOCKER" != "true" ]]; then
	ln -s $BVTSDK_DIR $CATKIN_DIR/src/mil_common/drivers/mil_blueview_driver/bvtsdk
fi


#====================================#
# SubjuGator Dependency Installation #
#====================================#

if [[ "$INSTALL_SUB" == "true" ]]; then
	instlog "Installing SubjuGator dependencies from the $REQUIRED_OS_ID repositories"

	# Communication pipe for the navigation vessel
	sudo apt-get install -qq socat

	# Optical character recognition
	sudo apt-get install -qq tesseract-ocr

	instlog "Installing SubjuGator ROS dependencies"

	# Controller
	sudo apt-get install -qq ros-$ROS_VERSION-control-toolbox
	sudo apt-get install -qq ros-$ROS_VERSION-controller-manager
	sudo apt-get install -qq ros-$ROS_VERSION-transmission-interface
	sudo apt-get install -qq ros-$ROS_VERSION-joint-limits-interface

	# Trajectory Generation
	sudo apt-get install -qq ros-$ROS_VERSION-ompl

	# 3D Mouse
	sudo apt-get install -qq ros-$ROS_VERSION-spacenav-node
fi


#===================================#
# NaviGator Dependency Installation #
#===================================#

if [[ "$INSTALL_NAV" == "true" ]]; then
	instlog "Installing NaviGator dependencies from the $REQUIRED_OS_ID repositories"

	# Compiler tools
	sudo apt-get install -qq build-essential
	sudo apt-get install -qq autoconf
	sudo apt-get install -qq automake
	sudo apt-get install -qq binutils-dev

	# Terry Guo's ARM toolchain
	sudo apt-get install -qq gcc-arm-none-eabi

	# Hardware drivers
	sudo apt-get install -qq ros-$ROS_VERSION-camera1394

	# Visualization
	sudo apt-get install -qq qt5-default

	instlog "Installing NaviGator ROS dependencies"

	# Serial communications
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial-arduino

	# Trajectory Generation
	sudo apt-get install -qq ros-$ROS_VERSION-ompl

	instlog "Performing setup tasks for lqRRT"
	cd $CATKIN_DIR/src/NaviGator/gnc/lqRRT
	sudo python setup.py build
	sudo python setup.py install

	# Pull large project files from Git-LFS
	instlog "Pulling large files for NaviGator"
	cd $CATKIN_DIR/src/NaviGator
	git lfs pull
fi


#=========================#
# Bashrc Alias Management #
#=========================#

# Generates the configuration directory if it does not exist
mkdir -p $MIL_CONFIG_DIR

# Write the MIL runcom file for sourcing all of the required project configurations
echo "#!/bin/bash" > $MILRC_FILE
echo "" > $MILRC_FILE
echo "# This file is created by the install script to source all of the configurations" > $MILRC_FILE
echo "# needed to work on the installed projects. Do not edit this file manually! Your" >> $MILRC_FILE
echo "# changes will be overwritten the next time the install script is run. Please" >> $MILRC_FILE
echo "# use the script to make changes." >> $MILRC_FILE

# Add variables for the core project directories
echo "" >> $MILRC_FILE
echo "export CATKIN_DIR=$CATKIN_DIR" >> $MILRC_FILE
echo "export MIL_CONFIG_DIR=$MIL_CONFIG_DIR" >> $MILRC_FILE

# Add CUDA programs to the path if the Nvidia repository was used to install it
if [[ "$INSTALL_CUDA" == "true" ]]; then
	if [[ "$REQUIRED_OS_ID" == "Ubuntu" ]]; then
		echo "" >> $MILRC_FILE
		echo "# Adds CUDA programs and libraries to the shell's path" >> $MILRC_FILE
		echo "export PATH=\$PATH:/usr/local/cuda/bin" >> $MILRC_FILE
		echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/cuda/lib64" >> $MILRC_FILE
	fi
fi

# Source ROS configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for ROS" >> $MILRC_FILE
echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $MILRC_FILE

# Source the workspace's configurations for bash on client machines
if [[ "$DOCKER" != "true" ]]; then
	echo "" >> $MILRC_FILE
	echo "# Sets up the shell environment for the catkin workspace" >> $MILRC_FILE
	echo "source \$CATKIN_DIR/devel/setup.bash" >> $MILRC_FILE
fi

# Source the project configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for each installed project" >> $MILRC_FILE
echo "for FILE in \$CATKIN_DIR/src/*; do" >> $MILRC_FILE
echo "	if [[ -f \$FILE/scripts/bash_aliases.sh ]]; then" >> $MILRC_FILE
echo "		source \$FILE/scripts/bash_aliases.sh" >> $MILRC_FILE
echo "	fi" >> $MILRC_FILE
echo "done" >> $MILRC_FILE

# Source MIL configurations for bash on this user account
if [[ -z "$(cat $BASHRC_FILE | grep $MILRC_FILE)" ]]; then
	echo "" >> $BASHRC_FILE
	echo "# Sets up the shell environment for installed MIL projects" >> $BASHRC_FILE
	echo "source $MILRC_FILE" >> $BASHRC_FILE
fi


#===========================#
# Catkin Workspace Building #
#===========================#

# Attempt to build the selected MIL software stack on client machines
if [[ "$DOCKER" != "true" ]]; then
	instlog "Building selected MIL software stack with catkin_make"
	catkin_make -C $CATKIN_DIR -B
fi

# Inform the user that the script has run to completion
instlog "The installation is complete"
instlog "A reboot may be required for some of the changes to take effect"
