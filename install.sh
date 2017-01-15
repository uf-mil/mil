#!/bin/bash
GOODCOLOR='\033[1;36m'
WARNCOLOR='\033[1;31m'
NOCOLOR='\033[0m'
GOODPREFIX="${GOODCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"

# Sane installation defaults for no argument cases
CATKIN_DIR=~/sub_ws
REQUIRED_OS="trusty"

instlog() {
    printf "$GOODPREFIX $@ $NOCOLOR\n"
}

instwarn() {
    printf "$WARNPREFIX $@ $NOCOLOR\n"
}

DTETCTED_OS="`lsb_release -sc`"
if ([ $DTETCTED_OS != $REQUIRED_OS ]); then
    instwarn "The Sub requires Ubuntu 14.04 (trusty)"
    instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
    exit 1
fi


#======================#
# Script Configuration #
#======================#

while [ "$#" -gt 0 ]; do
  case $1 in
    -h) printf "\nUsage: $0 \n
    [-c] catkin_workspace (Recommend: ~/sub_ws)\n
    example: ./install.sh -c ~/sub_ws
    \n"; exit ;;
    # TODO: Use this to check if catkin ws already set up
    -c) CATKIN_DIR="$2"
        shift 2;;
    -?) instwarn "Option $1 is not implemented"; exit ;;
  esac
done


#==================================#
# Repository and Dependancy Set Up #
#==================================#

# Make sure script dependencies are installed on bare bones installations
instlog "Installing install script dependencies"
sudo apt-get update -qq
sudo apt-get install -qq wget git

# Add software repositories for ROS and Gazebo
instlog "Adding ROS and Gazebo PPAs to software sources"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'

# Get the GPG signing keys for the above repositories
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Install ROS and other project dependencies
instlog "Installing ROS and Gazebo"
sudo apt-get update -qq
if (env | grep --quiet "SUB=true"); then
    sudo apt-get install -qq ros-indigo-desktop-full
else
    sudo apt-get install -qq ros-indigo-desktop
fi
sudo apt-get install -qq python-catkin-pkg python-rosdep gazebo7

# Sources ROS configurations for bash on this user account
source /opt/ros/indigo/setup.bash
if !(cat ~/.bashrc | grep --quiet "source /opt/ros"); then
    echo "" >> ~/.bashrc
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
    sudo rosdep init > /dev/null 2>&1
fi
rosdep update


#=====================================#
# Workspace and Sub Repository Set Up #
#=====================================#

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

# If we're in the Semaphore-ci, we should run catkin_make in the actual build thread
if (env | grep --quiet "SEMAPHORE=true"); then
    mv ~/Sub8 "$CATKIN_DIR/src"
fi

# Sources the workspace's configurations for bash on this user account
source "$CATKIN_DIR/devel/setup.bash"
if !(cat ~/.bashrc | grep --quiet "source $CATKIN_DIR/devel/setup.bash"); then
    echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
fi

# Check if the sub is set up; if it isn't, set it up
if !(ls "$CATKIN_DIR/src" | grep --quiet "Sub8"); then
    instlog "Looks like you don't have the sub set up, let's do that"
    cd "$CATKIN_DIR/src"
    git clone -q https://github.com/uf-mil/Sub8.git
    cd Sub8
    git remote rename origin upstream
    instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
fi

# Update the hosts file if necessary
if (env | grep --quiet "SUB=true"); then
    $CATKIN_DIR/src/Sub8/scripts/update-hosts.bash
fi

# Install external dependencies with another script
instlog "Running the get_dependencies.sh script to update external dependencies"
cd $CATKIN_DIR/src

INSTALL_FOLDER=$PWD;
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $DIR/bash_tools.sh


#======================#
# Script Configuration #
#======================#

instlog "Making sure we're in the catkin directory"

# Check if a file called CMakeLists.txt is present
if ! ls | grep --quiet CMakeLists.txt; then
    instwarn "Could not find a file called CMakeLists.txt"
    instwarn "Need to be in sub_ws/src, install failed"
    exit 1
fi

# Check if the working directory is src
if ! [ $(basename $PWD) == "src" ]; then
    instwarn "Need to be in sub_ws/src, install failed"
    exit 1
fi

#==================================#
# Repository and Dependancy Set Up #
#==================================#

instlog "Installing dependencies from the Ubuntu repositories"
sudo apt-get update -qq
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial
sudo apt-get install -qq python-twisted socat
sudo apt-get install -qq cmake binutils-dev python-pip
sudo apt-get install -qq libvtk5-dev python-vtk
sudo apt-get install -qq libboost-all-dev python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq libpcl-1.7-all libpcl-1.7-all-dev ros-indigo-pcl-conversions
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq libusb-1.0-0-dev

instlog "Installing ROS dependencies"
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control
sudo apt-get install -qq ros-indigo-control-toolbox ros-indigo-controller-manager ros-indigo-transmission-interface ros-indigo-joint-limits-interface ros-indigo-hardware-interface
sudo apt-get install -qq ros-indigo-sophus
sudo apt-get install -qq ros-indigo-driver-base
sudo apt-get install -qq ros-indigo-camera-info-manager
sudo apt-get install -qq ros-indigo-spacenav-node
sudo apt-get install -qq ros-indigo-camera1394
sudo apt-get install -qq ros-indigo-stereo-image-proc

instlog "Installing dependencies from Python PIP"
sudo pip install -q -U setuptools
sudo pip install -q -U mayavi > /dev/null 2>&1
sudo pip install -q -U argcomplete
sudo pip install -q -U scikit-learn > /dev/null 2>&1
sudo pip install -q -U tqdm

instlog "Getting ROS packages we need to install from source"
source $CATKIN_DIR/src/Sub8/scripts/bash_tools.sh
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/uf-mil/ros_alarms.git
ros_git_get "https://github.com/ros-simulation/gazebo_ros_pkgs.git --branch indigo-devel"
# catkin_make -C $INSTALL_FOLDER/..

# Attempt to build the sub from scratch on client machines
if !(env | grep --quiet "SEMAPHORE=true"); then
    instlog "Building the sub's software stack with catkin_make"
    catkin_make -C "$CATKIN_DIR" -j8
fi
