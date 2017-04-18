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


#======================#
# Script Configuration #
#======================#

# Install no project by default, the user must select one
SELECTED="false"
INSTALL_SUB="false"
INSTALL_PRO="false"
INSTALL_NAV="false"

# Set sane defaults for other install parameters
BVTSDK_PASSWORD=""

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
if [[ ! -d $CATKIN_DIR/src/bvtsdk ]]; then
	echo "The BlueView SDK used to interface with the Teledyne imaging sonar is encrypted"
	echo "in order to protect the intellectual property of BlueView. If you will be doing"
	echo "work with the imaging sonar on your machine, it is recommended that you install"
	echo "this now. If not, you probably do not need to."
	echo -n "Do you wish to install the SDK? [y/N] " && read RESPONSE
	echo ""

	# If the user chooses to install the BlueView SDK, retrieve the password from them
	if [[ "$RESPONSE" == "Y" || "$RESPONSE" == "y" ]]; then
		echo "The SDK is encrypted with a password. You need to obtain this password from one"
		echo "of the senior members of MIL."
		echo -n "Encryption password: " && read -s BVTSDK_PASSWORD
		echo ""
		echo ""
	fi

	# Detect whether or not a CUDA toolkit has already been installed
	if [[ ! -z "$(dpkg-query -W -f='${Status}' cuda 2>/dev/null | grep 'ok installed')" ||
	      ! -z "$(dpkg-query -W -f='${Status}' nvidia-cuda-toolkit  2>/dev/null | grep 'ok installed')" ]]; then
		INSTALL_CUDA="true"
	fi
fi


#==================#
# Pre-Flight Check #
#==================#

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

if [[ "$(lsb_release -si)" == "Debian" ]]; then
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


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Source ROS configurations for bash
source /opt/ros/$ROS_VERSION/setup.bash

# Set up catkin workspace directory
if [[ ! -f $CATKIN_DIR/src/CMakeLists.txt ]]; then
	instlog "Generating catkin workspace at $CATKIN_DIR"
	mkdir -p $CATKIN_DIR/src
	cd $CATKIN_DIR/src
	catkin_init_workspace
	catkin_make -C $CATKIN_DIR -B
else
	instlog "Using existing catkin workspace at $CATKIN_DIR"
fi

# Source the workspace's configurations for bash
source $CATKIN_DIR/devel/setup.bash

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


#=========================#
# Dependency Installation #
#=========================#

# Git-LFS for models and other large files
cd $CATKIN_DIR
git lfs install --skip-smudge

# The BlueView SDK for the Teledyne imaging sonar
if [[ ! -z "$BVTSDK_PASSWORD" ]]; then
	instlog "Decrypting and installing the BlueView SDK"
	cd $CATKIN_DIR/src
	curl -s https://raw.githubusercontent.com/uf-mil/installer/master/bvtsdk.tar.gz.enc | \
	openssl enc -aes-256-cbc -d -pass file:<(echo -n $BVTSDK_PASSWORD) | tar -xpz
	if [[ ! -d $CATKIN_DIR/src/bvtsdk ]]; then
		instwarn "Terminating installation due to incorrect password for the BlueView SDK"
		exit 1
	fi
fi

# Pull large project files from Git-LFS
if [[ "$INSTALL_NAV" == "true" ]]; then
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

# Source the workspace's configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for the catkin workspace" >> $MILRC_FILE
echo "source \$CATKIN_DIR/devel/setup.bash" >> $MILRC_FILE

# Source the project configurations for bash
echo "" >> $MILRC_FILE
echo "# Sets up the shell environment for each installed project" >> $MILRC_FILE
echo "for FILE in \$CATKIN_DIR/src/*; do" >> $MILRC_FILE
echo "	if [[ -f \$FILE/scripts/bash_aliases.sh ]]; then" >> $MILRC_FILE
echo "		source \$FILE/scripts/bash_aliases.sh" >> $MILRC_FILE
echo "	fi" >> $MILRC_FILE
echo "done" >> $MILRC_FILE

# Source MIL configurations for bash on this user account
source $MILRC_FILE
if [[ -z "$(cat $BASHRC_FILE | grep $MILRC_FILE)" ]]; then
	echo "" >> $BASHRC_FILE
	echo "# Sets up the shell environment for installed MIL projects" >> $BASHRC_FILE
	echo "source $MILRC_FILE" >> $BASHRC_FILE
fi


#===========================#
# Catkin Workspace Building #
#===========================#

# Build the selected MIL software stack
instlog "Building selected MIL software stack with catkin_make"
catkin_make -C $CATKIN_DIR -B

# Inform the user that the script has run to completion
instlog "The installation is complete"
