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
<<<<<<< HEAD
sudo apt-get install -qq ros-indigo-moveit-full
=======
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq libusb-1.0-0-dev

instlog "Installing ROS dependencies"
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control
sudo apt-get install -qq ros-indigo-control-toolbox ros-indigo-controller-manager ros-indigo-transmission-interface ros-indigo-joint-limits-interface ros-indigo-hardware-interface
>>>>>>> uf-mil/master
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
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/ros-simulation/gazebo_ros_pkgs.git

# catkin_make -C $INSTALL_FOLDER/..
