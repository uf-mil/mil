#!/bin/bash
GOODCOLOR='\033[1;36m'
WARNCOLOR='\e[31m'
NC='\033[0m' # No Color
GOODPREFIX="${GOODCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}INSTALLER:"

instlog() {
    printf "$GOODPREFIX $@ $NC\n"
}

instwarn() {
    printf "$WARNPREFIX $@ $NC\n"
}


while [ "$#" -gt 0 ]; do
  case $1 in
    -h) printf "usage: $0 \n
    [-c] catkin_directory (Recommend ~/repos/sub_dependencies)\n
    [-d] dependencies_directory  (Recommend ~/repos/catkin_ws)\n
    example: ./install.sh -d ~/repos/sub_dependencies -c ~/repos/catkin_ws
    ..."; exit ;;
    # TODO: Use this to check if catkin ws already set up
    -c) CATKIN_DIR="$2"
        shift 2;;
    -d) DEPS_DIR="$2"
        shift 2;;
    -?) echo "error: option -$OPTARG is not implemented"; exit ;;
  esac
done

# Spooky activity to check if we are in semaphore
if ls ~/ | grep --quiet Sub8$; then
    DEPS_DIR=~/repos/sub_dependencies;
    CATKIN_DIR=~/repos/catkin_ws;
fi

instlog "Installing dependencies in $DEPS_DIR"
instlog "Generating catkin workspace (If needed) at $CATKIN_DIR"

sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe"
sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu trusty main universe" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -qq

####### Install ROS
instlog "Looks like ROS is not installed, let's take care of that"
sudo apt-get install -qq ros-indigo-desktop python-catkin-pkg python-rosdep
set -e
source /opt/ros/indigo/setup.bash
if ! cat ~/.bashrc | grep "source /opt/ros"; then
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
fi
sudo rosdep init
rosdep update

# Set up catkin directory
mkdir -p "$CATKIN_DIR/src"
cd "$CATKIN_DIR/src"
catkin_init_workspace
catkin_make -C "$CATKIN_DIR"

# Set up sub

if  ls ~/ | grep -i Sub8; then
    # Here, we assume we're in Semaphore-ci, and should run catkin_make in the actual build thread
    instlog "Found Sub8 in HOME, Assuming we're in Semaphore"
    mv ~/Sub8 "$CATKIN_DIR/src"
fi

source "$CATKIN_DIR/devel/setup.bash"
if ! cat ~/.bashrc | grep "grep source.*$CATKIN_DIR/devel/setup.bash"; then
    echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
fi

####### Check if the sub is set up, and if it isn't, set it up
instlog "Looks like you don't have the sub set up, let's do that"
# Make our sub stuff

# Check if Sub8 repository already exists (Semaphore does this)
instlog "Looking for Sub8 (Are we in Semaphore?)"

if ! ls "$CATKIN_DIR/src" | grep Sub8; then
    cd "$CATKIN_DIR/src"
    git clone -q https://github.com/uf-mil/Sub8.git
    cd Sub8
    git remote rename origin upstream
    instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
fi

cd $CATKIN_DIR/src
$CATKIN_DIR/src/Sub8/scripts/get_dependencies.sh -d $DEPS_DIR
