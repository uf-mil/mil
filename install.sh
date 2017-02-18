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
	if [ ! -z "${HOST_PING}" ]; then

		# Uses packet loss percentage to determine if the connection is strong
		if [ $HOST_PING -lt 25 ]; then

			# Will return true if ping was successful and packet loss was below 25%
			echo "true"
		fi
	fi
}


#=======================#
# Configurable Defaults #
#=======================#

CATKIN_DIR=~/mil_ws
BASHRC_FILE=~/.bashrc
MILRC_FILE=~/.milrc

#======================#
# Script Configuration #
#======================#

# Install no project by default, the user must select one
SELECTED=false
INSTALL_SUB=false
INSTALL_PRO=false
INSTALL_NAV=false

# Set sane defaults for other install parameters
BVSDK_PASSWORD=""
ENABLE_USB_CAM=false
INSTALL_CUDA=false

# Use environment variables to determine which project to install on Semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	if (env | grep INSTALL_SUB | grep --quiet -oe '[^=]*$'); then
		INSTALL_SUB=true
	elif (env | grep INSTALL_PRO | grep --quiet -oe '[^=]*$'); then
		INSTALL_PRO=true
	elif (env | grep INSTALL_NAV | grep --quiet -oe '[^=]*$'); then
		INSTALL_NAV=true
	fi
	BVSDK_PASSWORD="`env | grep BVSDK_PASSWORD | grep -oe '[^=]*$'`"
	ENABLE_USB_CAM=false
	INSTALL_CUDA=true

else
	# Prompt the user to enter a catkin workspace to use
	echo "Catkin is the ROS build system and it combines CMake macros and Python scripts."
	echo "The catkin workspace is the directory where all source and build files for the"
	echo "project are stored. Our default is in brackets below, press enter to use it."
	echo -n "What catkin workspace should be used? [$CATKIN_DIR]: " && read RESPONSE
	echo ""
	if [ "$RESPONSE" != "" ]; then
		CATKIN_DIR=${RESPONSE/\~//home/$USER}
	fi

	echo "We use a forking workflow to facilitate code contributions on Github. This means"
	echo "that each user forks the main repository and has their own copy. In the"
	echo "repositories that we clone for projects, the main repository will be the"
	echo "'upstream' remote and your local fork will be the 'origin' remote. You should"
	echo "specify a fork URI for each repository you plan to push code to; otherwise,"
	echo "leave the field blank. These can also be set manually using this command:"
	echo "git remote add <remote_name> <user_fork_url>"
	echo -n "User fork URI for the software-common repository: " && read SWC_USER_FORK
	echo ""

	# Prompt the user to select a project to install
	while !($SELECTED); do
		echo "A MIL project must be selected for install"
		echo "	1. SubjuGator"
		echo "	2. PropaGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo "	3. NaviGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo -n "Project selection: " && read RESPONSE
		echo ""
		case "$RESPONSE" in
			"1")
				echo -n "User fork URI for the Sub8 repository: " && read SUB_USER_FORK
				INSTALL_SUB=true
				SELECTED=true
			;;
			"2")
				echo "The PropaGator project has not been worked on since the dark ages of MIL, so it"
				echo "is not supported by this script."
				echo ""
			;;
			"3")
				echo "The NaviGator project was developed on Ubuntu 14.04 with ROS Indigo. Several"
				echo "dependencies no longer exist in ROS Kinetic, so in order to install it, ROS"
				echo "Indigo, the Sub8 repository at an earlier date,  and all of the old Sub8"
				echo "dependencies will need to be downloaded and installed."
				echo -n "Do you still wish to proceed? [y/N] " && read RESPONSE
				echo ""
				if ([ "$RESPONSE" = "Y" ] || [ "$RESPONSE" = "y" ]); then
					echo -n "User fork URI for the Navigator repository: " && read NAV_USER_FORK
					INSTALL_NAV=true
					SELECTED=true
				fi
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
	if [ ! -d $CATKIN_DIR/src/bvtsdk ]; then
		echo "The BlueView SDK used to interface with the Teledyne imaging sonar is encrypted"
		echo "in order to protect the intellectual property of BlueView. If you will be doing"
		echo "work with the imaging sonar on your machine, it is recommended that you install"
		echo "this now. If not, you probably do not need to."
		echo -n "Do you wish to install the SDK? [y/N] " && read RESPONSE
		echo ""

		# If the user chooses to install the BlueView SDK, retrieve the password from them
		if ([ "$RESPONSE" = "Y" ] || [ "$RESPONSE" = "y" ]); then
			echo "The SDK is encrypted with a password. You need to obtain this password from one"
			echo "of the senior members of MIL."
			echo -n "Encryption password: " && read -s BVSDK_PASSWORD
			echo ""
			echo ""
		fi
	fi

	# Warn users about the security risks associated with enabling USB cameras before doing it
	if [ ! -f /etc/udev/rules.d/40-pgr.rules ]; then
		echo "MIL projects use Point Grey machine vision cameras for perception. A user only"
		echo "needs to enable access to USB cameras if they intend to connect one directly"
		echo "to their machine, which is unlikely. In order for a user to access a USB"
		echo "camera, a udev rule needs to be added that gives a group access to the hardware"
		echo "device on the camera. Long story short, this creates a fairly significant"
		echo "security hole on the machine that goes beyond the OS to actual device firmware."
		echo -n "Do you want to enable access to USB cameras? [y/N] " && read RESPONSE
		echo ""
		if ([ "$RESPONSE" = "Y" ] || [ "$RESPONSE" = "y" ]); then
			ENABLE_USB_CAM=true
		fi
	fi

	# Give users the option to install the CUDA toolkit if an Nvidia card is detected
	if !($INSTALL_NAV) && (lspci | grep --quiet "VGA compatible controller: NVIDIA"); then
		echo "An Nvidia graphics card was detected on this machine. Some of the perception"
		echo "packages may run faster with or require CUDA parallel processing on the GPU."
		echo "CUDA is not required, but the script can install it automatically. Ubuntu 16.04"
		echo "supports the 7.5 version natively; however, the 8.0 version can be installed"
		echo "through a repository maintained by Nvidia and runs much smoother. Since the 8.0"
		echo "version seems fairly stable, this is the version that the script will install."
		echo "The 7.5 version can still be installed manually with the following command:"
		echo "sudo apt-get install nvidia-cuda-toolkit"
		echo "Do you want to install CUDA Toolkit 8.0? [y/N] "
		echo ""
		if ([ "$RESPONSE" = "Y" ] || [ "$RESPONSE" = "y" ]); then
			INSTALL_CUDA=true
		fi
	fi
fi

if !($INSTALL_NAV); then
	REQUIRED_OS_CODENAME="xenial"
	REQUIRED_OS_VERSION="16.04"
	ROS_VERSION="kinetic"
else
	REQUIRED_OS_CODENAME="trusty"
	REQUIRED_OS_VERSION="14.04"
	ROS_VERSION="indigo"
fi


#==================#
# Pre-Flight Check #
#==================#

instlog "Acquiring root privileges"

# Gets the user to enter their password here instead of in the middle of the pre-flight check
# Purely here for aesthetics and to satisfy OCD
sudo true

instlog "Starting the pre-flight system check to ensure installation was done properly"

# Check whether or not github.com is reachable
# This also makes sure that the user is connected to the internet
if [ "`check_host github.com`" = "true" ]; then
	NET_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	NET_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Internet connectivity check"

if !($NET_CHECK); then

	# The script will not allow the user to install without internet
	instwarn "Terminating installation due to the lack of an internet connection"
	instwarn "The install script needs to be able to connect to Github and other sites"
	exit 1
fi

# Make sure script dependencies are installed quietly on bare bones installations
sudo apt-get update -qq
sudo apt-get install -qq lsb-release python-pip git build-essential > /dev/null 2>&1

# Ensure that the correct OS is installed
DETECTED_OS_CODENAME="`lsb_release -sc`"
if [ $DETECTED_OS_CODENAME = $REQUIRED_OS_CODENAME ]; then
	OS_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	OS_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "OS distribution and version check"

# Prevent the script from being run as root
if [ $USER != "root" ]; then
	ROOT_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	ROOT_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "User permissions check"

# Ensure that no ROS version is being sourced in the user's bash runcom file
if !(cat $BASHRC_FILE | grep --quiet "source /opt/ros"); then
	BASHRC_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	BASHRC_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Bash runcom file check"

if !($OS_CHECK); then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DETECTED_OS_CODENAME)"
	instwarn "MIL projects require Ubuntu $REQUIRED_OS_VERSION ($REQUIRED_OS_CODENAME)"
	exit 1
fi

if !($ROOT_CHECK); then

	# The script will not allow the user to install as root
	instwarn "Terminating installation due to forbidden user account"
	instwarn "The install script should not be run as root"
	exit 1
fi

if !($BASHRC_CHECK); then

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
instlog "Adding ROS PPA to software sources"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list'
sudo sh -c 'echo "deb-src http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Add software repository for Git-LFS to software sources
instlog "Adding the Git-LFS packagecloud repository to software sources"
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

# Add software repository for Nvidia to software sources
if ($INSTALL_CUDA); then
	instlog "Adding Nvidia PPA to software sources"
	sudo sh -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" >> /etc/apt/sources.list.d/cuda.list'
	wget -q -O - http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub | sudo apt-key add -
fi

# Install ROS and a few ROS dependencies
instlog "Installing ROS $(tr '[:lower:]' '[:upper:]' <<< ${ROS_VERSION:0:1})${ROS_VERSION:1}"
sudo apt-get update -qq
sudo apt-get install -qq ros-$ROS_VERSION-desktop-full

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update

# Source ROS configurations for bash
source /opt/ros/$ROS_VERSION/setup.bash


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Set up catkin workspace directory
if !([ -f $CATKIN_DIR/src/CMakeLists.txt ]); then
	instlog "Generating catkin workspace at $CATKIN_DIR"
	mkdir -p $CATKIN_DIR/src
	cd $CATKIN_DIR/src
	catkin_init_workspace
	catkin_make -C $CATKIN_DIR -B
else
	instlog "Using existing catkin workspace at $CATKIN_DIR"
fi

# Move the cloned git repository to the catkin workspace in semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	if [ -d ~/Sub8 ]; then
		mv ~/Sub8 $CATKIN_DIR/src
	elif [ -d ~/Navigator ]; then
		mv ~/Navigator $CATKIN_DIR/src
	fi
fi

# Source the workspace's configurations for bash
source $CATKIN_DIR/devel/setup.bash

# Download the software-common repository if it has not already been downloaded
if !(ls $CATKIN_DIR/src | grep --quiet "software-common"); then
	instlog "Downloading the software-common repository"
	cd $CATKIN_DIR/src
	git clone --recursive -q https://github.com/uf-mil/software-common.git
	cd $CATKIN_DIR/src/software-common
	git remote rename origin upstream
	if [ ! -z "$SWC_USER_FORK" ]; then
		git remote add origin "$SWC_USER_FORK"
	fi
fi

# Download the Sub8 repository if it has not already been downloaded and was selected for installation
if ($INSTALL_SUB) && !(ls $CATKIN_DIR/src | grep --quiet "Sub8"); then
	instlog "Downloading the Sub8 repository"
	cd $CATKIN_DIR/src
	git clone --recursive -q https://github.com/uf-mil/Sub8.git
	cd $CATKIN_DIR/src/Sub8
	git remote rename origin upstream
	if [ ! -z "$SUB_USER_FORK" ]; then
		git remote add origin "$SUB_USER_FORK"
	fi
fi

# Download the Navigator repository if it has not already been downloaded and was selected for installation
if ($INSTALL_NAV) && !(ls $CATKIN_DIR/src | grep --quiet "Navigator"); then
	instlog "Downloading the Sub8 repository"
	cd $CATKIN_DIR/src
	git clone --recursive -q https://github.com/uf-mil/Sub8.git
	cd $CATKIN_DIR/src/Sub8
	instlog "Rolling back the Sub8 repository; do not pull the latest version!"
	git reset --hard 0089e68b9f48b96af9c3821f356e3a487841e87e
	git remote remove origin
	instlog "Downloading the Navigator repository"
	cd $CATKIN_DIR/src
	git clone --recursive -q https://github.com/uf-mil/Navigator.git
	cd $CATKIN_DIR/src/Navigator
	git remote rename origin upstream
	if [ ! -z "$NAV_USER_FORK" ]; then
		git remote add origin "$NAV_USER_FORK"
	fi
fi


#================================#
# Common Dependency Installation #
#================================#

if ($ENABLE_USB_CAM); then
	instlog "Enabling USB cameras (a reboot is required for this to take full effect)"
	sudo usermod -a -G dialout "$USER"
	sudo groupadd --gid 999 pgrimaging
	sudo usermod -a -G pgrimaging "$USER"
	sudo wget -q https://raw.githubusercontent.com/uf-mil/installer/master/40-pgr.rules -O /etc/udev/rules.d/40-pgr.rules
	sudo service udev restart
fi

instlog "Installing common dependencies from the Ubuntu repositories"

# Cuda Toolkit 8.0
if ($INSTALL_CUDA); then
	sudo apt-get install -qq cuda
fi

# Scientific and technical computing
sudo apt-get install -qq python-scipy

# System tools
sudo apt-get install -qq tmux
sudo apt-get install -qq htop
sudo apt-get install -qq sshfs

# Git-LFS for models and other large files
sudo apt-get install -qq git-lfs
cd $CATKIN_DIR
git lfs install --skip-smudge

# Debugging utility
sudo apt-get install -qq gdb

# Machine Learning
sudo apt-get install -qq python-sklearn

# Visualization
sudo apt-get install -qq mayavi2

instlog "Installing common ROS dependencies"

# Hardware drivers
sudo apt-get install -qq ros-$ROS_VERSION-pointgrey-camera-driver

instlog "Installing common dependencies from Python PIP"

# Communication tool for the hydrophone board
sudo pip install -q -U crc16

# Visualization
sudo pip install -q -U tqdm

# The BlueView SDK for the Teledyne imaging sonar
if [ ! -z "$BVSDK_PASSWORD" ]; then
	instlog "Decrypting and installing the BlueView SDK"
	cd $CATKIN_DIR/src
	curl -s https://raw.githubusercontent.com/uf-mil/installer/master/bvtsdk.tar.gz.enc | \
	openssl enc -aes-256-cbc -d -pass file:<(echo -n $BVSDK_PASSWORD) | tar -xpz
fi


#==============================#
# Sub8 Dependency Installation #
#==============================#

if ($INSTALL_SUB); then
	instlog "Installing Sub8 dependencies from the Ubuntu repositories"

	# Communication pipe for the navigation vessel
	sudo apt-get install -qq socat

	# Optical character recognition
	sudo apt-get install -qq tesseract-ocr

	instlog "Installing Sub8 ROS dependencies"

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
# Navigator Dependency Installation #
#===================================#

if ($INSTALL_NAV); then
	instlog "Installing Navigator dependencies from the Ubuntu repositories"

	# Terry Guo's ARM toolchain
	sudo mkdir -p /etc/apt/preferences.d
	sudo sh -c "echo 'Package: *\nPin: origin "ppa.launchpad.net"\nPin-Priority: 999' > /etc/apt/preferences.d/arm"
	sudo sh -c 'echo "deb http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu trusty main" > /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo sh -c 'echo "deb-src http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu trusty main" > /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xA3421AFB
	sudo apt-get update -qq
	sudo apt-get install -qq gcc-arm-none-eabi

	instlog "Installing Navigator ROS dependencies"

	# Serial communications
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial-arduino

	# Thruster driver
	sudo apt-get install -qq ros-$ROS_VERSION-roboteq-driver

	instlog "Performing setup tasks for lqRRT"
	sudo python $CATKIN_DIR/src/Navigator/gnc/lqRRT/setup.py build
	sudo python $CATKIN_DIR/src/Navigator/gnc/lqRRT/setup.py install

	# Pull large project files from Git-LFS
	instlog "Pulling large files for Navigator"
	cd $CATKIN_DIR/src/Navigator
	git lfs pull
fi


#=========================#
# Bashrc Alias Management #
#=========================#

# Write the MIL runcom file for sourcing all of the required project configurations
echo "# This file is created by the install script to source all of the configurations" > $MILRC_FILE
echo "# needed to work on the installed projects. Do not edit this file manually! Your" >> $MILRC_FILE
echo "# changes will be overwritten the next time the install script is run. Please use" >> $MILRC_FILE
echo "# the script to make changes." >> $MILRC_FILE

# Source ROS configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for ROS" >> $MILRC_FILE
echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $MILRC_FILE

# Source the workspace's configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for the catkin workspace" >> $MILRC_FILE
echo "export CATKIN_DIR=$CATKIN_DIR" >> $MILRC_FILE
echo "source \$CATKIN_DIR/devel/setup.bash" >> $MILRC_FILE

# Source the project configurations for bash
declare -a ALIASED_REPOSITORIES=("software-common" "Sub8" "Navigator")
for REPOSITORY in "${ALIASED_REPOSITORIES[@]}"; do
	if [ -f $CATKIN_DIR/src/$REPOSITORY/scripts/bash_aliases.sh ]; then
		if !(cat $MILRC_FILE | grep --quiet "# Sets up the shell environment for each installed project"); then
			echo "" >> $MILRC_FILE
			echo "# Sets up the shell environment for each installed project" >> $MILRC_FILE
		fi
		echo "source \$CATKIN_DIR/src/$REPOSITORY/scripts/bash_aliases.sh"  >> $MILRC_FILE
	fi
done

# Source MIL configurations for bash on this user account
source $MILRC_FILE
if !(cat $BASHRC_FILE | grep --quiet "source $MILRC_FILE"); then
	echo "" >> $BASHRC_FILE
	echo "# Sets up the shell environment for installed MIL projects" >> $BASHRC_FILE
	echo "source $MILRC_FILE" >> $BASHRC_FILE
fi


#========================#
# Build Catkin Workspace #
#========================#

# Attempt to build the Navigator stack on client machines
if !(env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	instlog "Building MIL's software stack with catkin_make"
	catkin_make -C $CATKIN_DIR -B
fi
