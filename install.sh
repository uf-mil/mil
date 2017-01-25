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

	# Attempts to ping a host to make sure it is reachable
	HOST="$1"

	HOST_PING=$(ping -c 2 $HOST 2>&1 | grep "% packet" | cut -d" " -f 6 | tr -d "%")
	if ! [ -z "${HOST_PING}" ]; then

		# Uses packet loss percentage to determine if the connection is strong
		if [ $HOST_PING -lt 25 ]; then

			# Will return true if ping was successful and packet loss was below 25%
			echo "true"
		fi
	fi
}

ros_git_get() {
	# Usage example: ros_git_get https://github.com/uf-mil/software-common.git
	NEEDS_INSTALL=true;
	INSTALL_URL=$1;
	builtin cd $CATKIN_DIR/src

	# Check if it already exists
	for FOLDER in $CATKIN_DIR/src/*; do
		if ! [ -d $FOLDER ]; then
			continue;
		fi

		builtin cd $FOLDER
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
		git clone -q $INSTALL_URL --depth=100 --recursive
	fi
}


#=======================#
# Configurable Defaults #
#=======================#

CATKIN_DIR=~/mil_ws
BASHRC_FILE=~/.bashrc


#======================#
# Script Configuration #
#======================#

# Install no project by default, the user must select one
INSTALL_SUB=false
INSTALL_PRO=false
INSTALL_NAV=false

# Use environment variables to determine which project to install on Semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	if (env | grep INSTALL_SUB | grep --quiet -oe '[^=]*$'); then
		INSTALL_SUB=true
	elif (env | grep INSTALL_PRO | grep --quiet -oe '[^=]*$'); then
		INSTALL_PRO=true
	elif (env | grep INSTALL_NAV | grep --quiet -oe '[^=]*$'); then
		INSTALL_NAV=true
	fi

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

	# Prompt the user to select a project to install
	SELECTED=false
	while !($SELECTED); do
		echo "A MIL project must be selected for install"
		echo "	1. SubjuGator"
		echo "	2. PropaGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo "	3. NaviGator $(tput bold)[DEPRECATED]$(tput sgr0)"
		echo -n "Project selection: " && read RESPONSE
		echo ""
		case "$RESPONSE" in
			"1")
				if (cat $BASHRC_FILE | grep --quiet "source /opt/ros/indigo/setup.bash") && \
				   !(cat $BASHRC_FILE | grep --quiet "#source /opt/ros/indigo/setup.bash"); then
					instwarn "Terminating installation due to conflicting ROS versions"
					instwarn "SubjuGator requires ROS Kinetic, but ROS Indigo is being sourced"
					instwarn "To fix this issue, comment out this line in $BASHRC_FILE: source /opt/ros/indigo/setup.bash"
					exit 1
				else
					INSTALL_SUB=true
					SELECTED=true
				fi
			;;
			"2")
				echo "The PropaGator project has not been worked on since the dark ages of MIL, so"
				echo "it is not supported by this script."
				echo ""
			;;
			"3")
				echo "The NaviGator project was developed on Ubuntu 14.04 with ROS Indigo."
				echo "Several dependencies no longer exist in ROS Kinetic, so in order to install it,"
				echo "ROS Indigo, the Sub8 repository at an earlier date,  and all of the old Sub8"
				echo "dependencies will need to be downloaded and installed."
				echo -n "Do you still wish to proceed? [y/N] " && read RESPONSE
				echo ""
				if ([ "$RESPONSE" = "Y" ] || [ "$RESPONSE" = "y" ]); then
					if (cat $BASHRC_FILE | grep --quiet "source /opt/ros/kinetic/setup.bash") && \
					   !(cat $BASHRC_FILE | grep --quiet "#source /opt/ros/kinetic/setup.bash"); then
						instwarn "Terminating installation due to conflicting ROS versions"
						instwarn "NaviGator requires ROS Indigo, but ROS Kinetic is being sourced"
						instwarn "To fix this issue, comment out this line in $BASHRC_FILE: source /opt/ros/kinetic/setup.bash"
						exit 1
					else
						INSTALL_NAV=true
						SELECTED=true
					fi
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
fi

if ($INSTALL_SUB); then
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

if !($OS_CHECK); then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DETECTED_OS_CODENAME)"
	instwarn "MIL projects require Ubuntu $REQUIRED_OS_VERSION ($REQUIRED_OS_CODENAME)"
	exit 1
fi

if !($ROOT_CHECK); then

	# The script will not allow the user to install as root
	instwarn "Terminating installation due to elevated user permissions"
	instwarn "The install script should not be run as root"
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

# Install ROS and a few ROS dependencies
instlog "Installing ROS $(tr '[:lower:]' '[:upper:]' <<< ${ROS_VERSION:0:1})${ROS_VERSION:1}"
sudo apt-get update -qq
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	sudo apt-get install -qq ros-$ROS_VERSION-desktop
else
	sudo apt-get install -qq ros-$ROS_VERSION-desktop-full
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update

# Source ROS configurations for bash on this user account
source /opt/ros/$ROS_VERSION/setup.bash
if !(cat $BASHRC_FILE | grep --quiet "source /opt/ros/$ROS_VERSION/setup.bash"); then
	echo "" >> $BASHRC_FILE
	echo "# Sets up the shell environment for ROS" >> $BASHRC_FILE
	echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $BASHRC_FILE
fi


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

# Source the workspace's configurations for bash on this user account
source $CATKIN_DIR/devel/setup.bash
if !(cat $BASHRC_FILE | grep --quiet "source $CATKIN_DIR/devel/setup.bash"); then
	echo "" >> $BASHRC_FILE
	echo "# Sets up the shell environment for the $CATKIN_DIR workspace" >> $BASHRC_FILE
	echo "source $CATKIN_DIR/devel/setup.bash" >> $BASHRC_FILE
fi

# Download the software-common repository if it has not already been downloaded
if !(ls $CATKIN_DIR/src | grep --quiet "software-common"); then
	instlog "Downloading the software-common repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/software-common.git
	cd $CATKIN_DIR/src/software-common
	git remote rename origin upstream
fi

# Download the Sub8 repository if it has not already been downloaded and was selected for installation
if ($INSTALL_SUB) && !(ls $CATKIN_DIR/src | grep --quiet "Sub8"); then
	instlog "Downloading the Sub8 repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/Sub8.git
	cd $CATKIN_DIR/src/Sub8
	git remote rename origin upstream
	instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi

# Download the Navigator repository if it has not already been downloaded and was selected for installation
if ($INSTALL_NAV) && !(ls $CATKIN_DIR/src | grep --quiet "Navigator"); then
	instlog "Downloading the Sub8 repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/Sub8.git
	cd $CATKIN_DIR/src/Sub8
	instlog "Rolling back the Sub8 repository; do not pull the latest versiion!"
	git reset --hard 0089e68b9f48b96af9c3821f356e3a487841e87e
	git remote remove origin
	instlog "Downloading the Navigator repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/Navigator.git
	cd $CATKIN_DIR/src/Navigator
	git remote rename origin upstream
	instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi


#================================#
# Common Dependency Installation #
#================================#

instlog "Installing common dependencies from the Ubuntu repositories"

# System tools
sudo apt-get install -qq tmux
sudo apt-get install -qq sshfs

# Git-LFS for models and other large files
sudo apt-get install -qq git-lfs
cd $CATKIN_DIR
git lfs install --skip-smudge

# Debugging utility
sudo apt-get install -qq gdb

instlog "Installing common dependencies from Python PIP"

# Communication tool for the hydrophone board
sudo pip install -q -U crc16

# Machine Learning
sudo pip install -q -U scikit-learn > /dev/null 2>&1

# Visualization
sudo pip install -q -U mayavi > /dev/null 2>&1
sudo pip install -q -U tqdm

instlog "Cloning common Git repositories that need to be built"
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/ros_alarms


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
	instlog "Installing Sub8 dependencies from the Ubuntu repositories"

	# Terry Guo's ARM toolchain
	sudo mkdir -p /etc/apt/preferences.d
	sudo sh -c "echo 'Package: *\nPin: origin "ppa.launchpad.net"\nPin-Priority: 999' > /etc/apt/preferences.d/arm"
	sudo sh -c 'echo "deb http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu trusty main" > /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo sh -c 'echo "deb-src http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu trusty main" > /etc/apt/sources.list.d/gcc-arm-embedded.list'
	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xA3421AFB
	sudo apt-get update -qq
	sudo apt-get install -qq gcc-arm-none-eabi

	# Visualization
	sudo apt-get install -qq python-progressbar

	instlog "Installing Navigator ROS dependencies"

	# Serial communications
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial
	sudo apt-get install -qq ros-$ROS_VERSION-rosserial-arduino

	# Thruster driver
	sudo apt-get install -qq ros-$ROS_VERSION-roboteq-driver

	instlog "Cloning Navigator Git repositories that need to be built"

	# Software to interface with MIL hardware
	ros_git_get https://github.com/uf-mil-archive/hardware-common

	# Required steps to build and install lqRRT
	ros_git_get https://github.com/jnez71/lqRRT.git
	sudo python $CATKIN_DIR/src/lqRRT/setup.py build
	sudo python $CATKIN_DIR/src/lqRRT/setup.py install

	# Pull large project files from Git-LFS
	instlog "Pulling large files for Navigator"
	cd $CATKIN_DIR/src/Navigator
	git lfs pull
fi


#=========================#
# Bashrc Alias Management #
#=========================#

SWC_ALIASES=$CATKIN_DIR/src/software-common/scripts/bash_aliases.sh
SUB_ALIASES=$CATKIN_DIR/src/Sub8/scripts/bash_aliases.sh
NAV_ALIASES=$CATKIN_DIR/src/Navigator/scripts/bash_aliases.sh

# Source the software-common configurations for bash on this user account
source $SWC_ALIASES
if !(cat $BASHRC_FILE | grep --quiet "source $SWC_ALIASES"); then
	echo "" >> $BASHRC_FILE
	echo "# Adds MIL aliases to shell environment" >> $BASHRC_FILE
	echo "source $SWC_ALIASES" >> $BASHRC_FILE
fi

# Source the Sub8 configurations for bash on this user account
if ($INSTALL_SUB); then
	source $SUB_ALIASES
	if !(cat $BASHRC_FILE | grep --quiet "source $SUB_ALIASES"); then
		echo "" >> $BASHRC_FILE
		echo "# Adds SubjuGator aliases to shell environment" >> $BASHRC_FILE
		echo "source $SUB_ALIASES" >> $BASHRC_FILE
	fi
fi

# Source the Navigator configurations for bash on this user account
if ($INSTALL_NAV); then
	source $NAV_ALIASES
	if !(cat $BASHRC_FILE | grep --quiet "source $NAV_ALIASES"); then
		echo "" >> $BASHRC_FILE
		echo "# Adds NaviGator aliases to shell environment" >> $BASHRC_FILE
		echo "source $NAV_ALIASES" >> $BASHRC_FILE
	fi
fi


#==========================#
# Finalization an Clean Up #
#==========================#

# Attempt to build the Navigator stack on client machines
if !(env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	instlog "Building MIL's software stack with catkin_make"
	catkin_make -C $CATKIN_DIR -B
fi
