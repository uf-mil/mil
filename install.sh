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
			return `true`
		fi
	fi
	return `false`
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
		git clone -q $INSTALL_URL --depth=1 --recursive
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
elif ($INSTALL_NAV); then

	# Navigator currently depends on the Sub8 repository
	# This may change soon, but this will install Sub8 for now
	INSTALL_SUB=true
fi

# The path to the user's bash configuration file
BASHRC_FILE=~/.bashrc

# Creates a temporary file for building the new bashrc file
TMP_BASHRC=$(mktemp /tmp/update-bashrc.XXXXXX)

# Finds the line number of the block header if it exists in the file
MIL_LINE=`cat $BASHRC_FILE | grep "# Bash configurations for MIL" -n | cut -d ':' -f 1`

# Preserves all of the users other bash configurations (ones above the MIL header)
if [ -s $BASHRC_FILE ]; then

	# If there is no MIL block, copies the old bashrc file to the new bashrc file
	if [ -z "$MIL_LINE" ]; then
		USERS_BASHRC="`cat $BASHRC_FILE`"
		echo "$USERS_BASHRC" >> $TMP_BASHRC
		echo "" >> $TMP_BASHRC

	# If there is a MIL block, only copies the configurations above it
	elif [ $(($MIL_LINE - 2)) -gt 0 ]; then
		USERS_BASHRC="`head -$(($MIL_LINE - 2)) $BASHRC_FILE`"
		echo "$USERS_BASHRC" >> $TMP_BASHRC
		echo "" >> $TMP_BASHRC
	fi
fi


#==================#
# Pre-Flight Check #
#==================#

instlog "Starting the pre-flight system check to ensure installation was done properly"

# The lsb-release package is critical to check the OS version
# It may not be on bare-bones systems, so it is installed here if necessary
sudo apt-get update -qq
sudo apt-get install -qq lsb-release

# Ensure that the correct OS is installed
DTETCTED_OS="`lsb_release -sc`"
if [ $DTETCTED_OS = $REQUIRED_OS ]; then
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
echo "Running user check"

# Check whether or not github.com is reachable
# This also makes sure that the user is connected to the internet
if (check_host "github.com"); then
	NET_CHECK=true
	echo -n "[ " && instpass && echo -n "] "
else
	NET_CHECK=false
	echo -n "[ " && instfail && echo -n "] "
fi
echo "Internet connectivity check"

if !($OS_CHECK); then

	# The script will not allow the user to install on an unsupported OS
	instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
	instwarn "MIL projects require Ubuntu 14.04 (trusty)"
	exit 1
fi

if !($ROOT_CHECK); then

	# The script will not allow the user to install as root
	instwarn "Terminating installation due to forbidden user"
	instwarn "The install script should not be run as root"
	exit 1
fi

if !($NET_CHECK); then

	# The script will not allow the user to install without internet
	instwarn "Terminating installation due to the lack of an internet connection"
	instwarn "The install script needs to be able to connect to GitHub and other sites"
	exit 1
fi


#===================================================#
# Repository and Set Up and Main Stack Installation #
#===================================================#

# Make sure script dependencies are installed on bare bones installations
instlog "Installing install script dependencies"
sudo apt-get install -qq wget curl aptitude fakeroot ssh git

# Add software repositories for ROS and Gazebo
instlog "Adding ROS and Gazebo PPAs to software sources"
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu trusty main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main\" > /etc/apt/sources.list.d/gazebo-latest.list"

# Get the GPG signing keys for the above repositories
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Add software repository for Git-LFS
instlog "Adding the Git-LFS packagecloud repository to software sources"
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

# Install ROS and other project dependencies
instlog "Installing ROS Indigo base packages"
sudo apt-get update -qq
sudo apt-get install -qq python-catkin-pkg python-rosdep
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	sudo apt-get install -qq ros-indigo-desktop
else
	sudo apt-get install -qq ros-indigo-desktop-full
fi

# Break the ROS Indigo metapackage and install an updated version of Gazebo
instlog "Installing the latest version of Gazebo"
sudo aptitude unmarkauto -q '?reverse-depends(ros-indigo-desktop-full) | ?reverse-recommends(ros-indigo-desktop-full)'
sudo apt-get purge -qq ros-indigo-gazebo*
sudo apt-get install -qq gazebo7
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control

# Source ROS configurations for bash on this user account
source /opt/ros/indigo/setup.bash
if !(cat $TMP_BASHRC | grep --quiet "source /opt/ros"); then
	echo "" >> $TMP_BASHRC
	echo "# Sets up the shell environment for ROS" >> $TMP_BASHRC
	echo "source /opt/ros/indigo/setup.bash" >> $TMP_BASHRC
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
	sudo rosdep init > /dev/null 2>&1
fi
rosdep update


#=================================#
# Workspace and Repository Set Up #
#=================================#

# Set up catkin workspace directory
if !([ -f $CATKIN_DIR/src/CMakeLists.txt ]); then
	instlog "Generating catkin workspace at $CATKIN_DIR"
	mkdir -p "$CATKIN_DIR/src"
	cd "$CATKIN_DIR/src"
	catkin_init_workspace
	catkin_make -C "$CATKIN_DIR" -B
else
	instlog "Using existing catkin workspace at $CATKIN_DIR"
fi

# Move the cloned git repository to the catkin workspace in semaphore
if (env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	if [ -d ~/Navigator ]; then
		mv ~/Navigator "$CATKIN_DIR/src"
	elif [ -d ~/Sub8 ]; then
		mv ~/Sub8 "$CATKIN_DIR/src"
	fi
fi

# Source the workspace's configurations for bash on this user account
source "$CATKIN_DIR/devel/setup.bash"
if !(cat $TMP_BASHRC | grep --quiet "source $CATKIN_DIR/devel/setup.bash"); then
	echo "source $CATKIN_DIR/devel/setup.bash" >> $TMP_BASHRC
fi

# Check if the Navigator repository is present; if it isn't, download it
if ($INSTALL_NAV) && !(ls "$CATKIN_DIR/src" | grep --quiet "Navigator"); then
	instlog "Downloading the Navigator repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/Navigator.git
	cd $CATKIN_DIR/src/Navigator
	git remote rename origin upstream
	instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi

# Check if the Sub8 repository is present; if it isn't, download it
if ($INSTALL_SUB) && !(ls "$CATKIN_DIR/src" | grep --quiet "Sub8"); then
	instlog "Downloading the Sub8 repository"
	cd $CATKIN_DIR/src
	git clone -q https://github.com/uf-mil/Sub8.git
	cd $CATKIN_DIR/src/Sub8
	git remote rename origin upstream
	instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi


#================================#
# Common Dependency Installation #
#================================#

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
sudo apt-get install -qq qt5-default
sudo apt-get install -qq qt5-qmake

# Tools
sudo apt-get install -qq sshfs
sudo apt-get install -qq git-lfs gitk
git lfs install --skip-smudge
sudo apt-get install -qq tmux

# Tools needed for hardware-common
sudo mkdir -p /etc/apt/preferences.d/
sudo bash -c "echo -e 'Package: *\nPin: origin "ppa.launchpad.net"\nPin-Priority: 999' > /etc/apt/preferences.d/arm"
sudo rm -f /etc/apt/sources.list.d/terry_guo-gcc-arm-embedded-*.list
sudo add-apt-repository -y ppa:terry.guo/gcc-arm-embedded
sudo apt-get update -qq
sudo apt-get remove -qq -y gcc-arm-none-eabi binutils-arm-none-eabi
sudo apt-get remove -qq -y libnewlib-arm-none-eabi libnewlib-dev
sudo apt-get install -qq -y --force-yes gcc-arm-none-eabi
sudo apt-get install -qq autoconf automake libtool

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
sudo pip install -q -U progressbar

# Machine Learning
sudo pip install -q -U scikit-learn > /dev/null 2>&1

# Visualization
sudo pip install -q -U mayavi > /dev/null 2>&1

instlog "Cloning common Git repositories that need to be built"
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/ros-simulation/gazebo_ros_pkgs.git
ros_git_get https://github.com/uf-mil/hardware-common.git


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

	# Message types
	sudo apt-get install -qq ros-indigo-tf2-sensor-msgs ros-indigo-tf2-geometry-msgs

	# Pulling large project files from Git-LFS
	instlog "Pulling large files for Navigator"
	cd $CATKIN_DIR/src/Navigator
	git lfs pull

	instlog "Cloning Navigator Git repositories that need to be built"
	ros_git_get https://github.com/jnez71/lqRRT.git
	ros_git_get https://github.com/gareth-cross/rviz_satellite.git

	# Required steps to build and install lqRRT
	sudo python $CATKIN_DIR/src/lqRRT/setup.py build
	sudo python $CATKIN_DIR/src/lqRRT/setup.py install
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

#=========================#
# Bashrc Alias Management #
#=========================#

BASHRC_STR="
# Bash configurations for MIL (any additions below this block will be deleted)
# These are the hostnames for all devices that run a remote roscore
SUB_HOST=mil-sub-sub8.ad.mil.ufl.edu
WMV_HOST=mil-nav-wamv.ad.mil.ufl.edu
PRC_HOST=mil-nav-perception.ad.mil.ufl.edu
SHT_HOST=mil-shuttle.ad.mil.ufl.edu
JN5_HOST=mil-johnny-five.ad.mil.ufl.edu

check_host() {

	# Attempts to ping a host to make sure it is reachable
	HOST=\"\$1\"

	HOST_PING=\$(ping -w 1 -c 2 \$HOST 2>&1 | grep \"% packet\" | cut -d\" \" -f 6 | tr -d \"%\")
	if ! [ -z \"\${HOST_PING}\" ]; then

		# Uses packet loss percentage to determine if the connection is strong
		if [ \$HOST_PING -lt 25 ]; then

			# Will return true if ping was successful and packet loss was below 25%
			return \`true\`
		fi
	fi
	return \`false\`
}

check_connection() {
	if (check_host \"\$SUB_HOST\"); then
		SUB_CHECK=true
	else
		SUB_CHECK=false
	fi
	if (check_host \"\$WMV_HOST\"); then
		WMV_CHECK=true
	else
		WMV_CHECK=false
	fi
	if (check_host \"\$PRC_HOST\"); then
		PRC_CHECK=true
	else
		PRC_CHECK=false
	fi
	if (check_host \"\$SHT_HOST\"); then
		SHT_CHECK=true
	else
		SHT_CHECK=false
	fi
	if (check_host \"\$JN5_HOST\"); then
		JN5_CHECK=true
	else
		JN5_CHECK=false
	fi

	if ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	    [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	    [ \"\$JN5_CHECK\" = \"false\" ]); then
		echo \"None of the MIL roscores are available on this network\"
	else
		if [ \"\$SUB_CHECK\" = \"true\" ]; then
			echo \"SubjuGator is accessible on this network\"
		fi
		if [ \"\$WMV_CHECK\" = \"true\" ]; then
			echo \"Navigator WAMV is accessible on this network\"
		fi
		if [ \"\$PRC_CHECK\" = \"true\" ]; then
			echo \"Navigator Perception is accessible on this network\"
		fi
		if [ \"\$SHT_CHECK\" = \"true\" ]; then
			echo \"Shuttle is accessible on this network\"
		fi
		if [ \"\$JN5_CHECK\" = \"true\" ]; then
			echo \"Johnny Five is accessible on this network\"
		fi
	fi
}

set_ros_ip() {
	LOCAL_IP=\"\`ip route get 192.168.37.0/24 | awk '{print \$NF; exit}'\`\"
	LOCAL_HOSTNAME=\"\`hostname\`.ad.mil.ufl.edu\"

	# Sets ROS_HOSTNAME to the this machine's hostname
	export ROS_HOSTNAME=\$LOCAL_HOSTNAME

	# Sets ROS_IP to the IP on this machine's main NIC
	export ROS_IP=\$LOCAL_IP
}

unset_ros_ip() {

	# Unsets ROS_IP and ROS_HOSTNAME, which will default to localhost
	unset ROS_IP
	unset ROS_HOSTNAME
}

set_ros_master() {

	# Sets ROS_MASTER_URI to the hostname of the correct MIL roscore
	export ROS_MASTER_URI=\$REMOTE_ROSCORE_URI
	echo \"The master roscore is set to \$REMOTE_ROSCORE_URI\"
}

unset_ros_master() {

	# Sets ROS_MASTER_URI to point back to localhost
	export ROS_MASTER_URI=http://localhost:11311
	echo \"The master roscore is set to this machine\"
}

ros_connect() {
	check_connection

	# If none of the MIL roscores are accessible, use localhost as the default roscore
	if ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	    [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	    [ \"\$JN5_CHECK\" = \"false\" ]); then
		ros_disconnect

	# If just one MIL roscore was accessible, connect directly to that roscore
	elif ([ \"\$SUB_CHECK\" = \"true\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	      [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	      [ \"\$JN5_CHECK\" = \"false\" ]); then
		REMOTE_ROSCORE_URI=http://\$SUB_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"true\" ] &&\\
	      [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	      [ \"\$JN5_CHECK\" = \"false\" ]); then
		REMOTE_ROSCORE_URI=http://\$WMV_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	      [ \"\$PRC_CHECK\" = \"true\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	      [ \"\$JN5_CHECK\" = \"false\" ]); then
		REMOTE_ROSCORE_URI=http://\$PRC_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	      [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"true\" ] &&\\
	      [ \"\$JN5_CHECK\" = \"false\" ]); then
		REMOTE_ROSCORE_URI=http://\$SHT_HOST:11311
		set_ros_ip
		set_ros_master
	elif ([ \"\$SUB_CHECK\" = \"false\" ] && [ \"\$WMV_CHECK\" = \"false\" ] &&\\
	      [ \"\$PRC_CHECK\" = \"false\" ] && [ \"\$SHT_CHECK\" = \"false\" ] &&\\
	      [ \"\$JN5_CHECK\" = \"true\" ]); then
		REMOTE_ROSCORE_URI=http://\$JN5_HOST:11311
		set_ros_ip
		set_ros_master

	# If multiple roscores were reachable, allow the user to select one
	else
		echo \"\"
		echo \"Multiple MIL roscores were detected!\"
		if ([ \"\$SUB_CHECK\" = \"true\" ]); then
			echo \"	1. SubjuGator\"
		fi
		if ([ \"\$WMV_CHECK\" = \"true\" ]); then
			echo \"	2. Navigator WAMV\"
		fi
		if ([ \"\$PRC_CHECK\" = \"true\" ]); then
			echo \"	3. Navigator Perception\"
		fi
		if ([ \"\$SHT_CHECK\" = \"true\" ]); then
			echo \"	4. Shuttle\"
		fi
		if ([ \"\$JN5_CHECK\" = \"true\" ]); then
			echo \"	5. Johnny Five\"
		fi
		echo \"\"
		echo -n \"Select a roscore to connect to: \"
		read SELECTION

		if [ \"\$SELECTION\" = \"1\" ]; then
			REMOTE_ROSCORE_URI=http://\$SUB_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ \"\$SELECTION\" = \"2\" ]; then
			REMOTE_ROSCORE_URI=http://\$WMV_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ \"\$SELECTION\" = \"3\" ]; then
			REMOTE_ROSCORE_URI=http://\$PRC_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ \"\$SELECTION\" = \"4\" ]; then
			REMOTE_ROSCORE_URI=http://\$SHT_HOST:11311
			set_ros_ip
			set_ros_master
		elif [ \"\$SELECTION\" = \"5\" ]; then
			REMOTE_ROSCORE_URI=http://\$JN5_HOST:11311
			set_ros_ip
			set_ros_master
		else
			echo \"Invalid selection value, no roscore selected\"
		fi
	fi
}

ros_disconnect() {

	# Disconnects from any remote roscore and connects to the local one
	unset_ros_ip
	unset_ros_master
}

# Prints debugging output for the master roscore that is currently selected
alias rosenv='echo \"ROS_IP=\$ROS_IP
ROS_HOSTNAME=\$ROS_HOSTNAME
ROS_MASTER_URI=\$ROS_MASTER_URI\"'"

# Writes the bashrc alias string to the new bashrc file
echo "$BASHRC_STR" >> $TMP_BASHRC

# Copies all entries in the new bashrc file to the machine's bashrc file
cat $TMP_BASHRC > $BASHRC_FILE

# Removes the temporary file used for building the new bashrc file
rm -rf $TMP_BASHRC

instlog "The bashrc file has been updated with the current aliases"


#==========================#
# Finalization an Clean Up #
#==========================#

# Attempt to build the Navigator stack on client machines
if !(env | grep SEMAPHORE | grep --quiet -oe '[^=]*$'); then
	instlog "Building MIL's software stack with catkin_make"
	catkin_make -C "$CATKIN_DIR" -B
fi

# Remove the initial install script if it was not in the Navigator repository
if !(echo "$SCRIPT_DIR" | grep --quiet "src/Navigator"); then
	rm -f "$SCRIPT_PATH"
fi
