INSTALL_FOLDER=$PWD;
DEPS_DIR=$PWD
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

while [ "$#" -gt 0 ]; do
  case $1 in
    -h) printf "usage: $0 \n
    [-d] dependencies_directory  (Recommend ~/repos/catkin_ws)\n
    example: ./install.sh -d ~/repos/sub_dependencies
    ..."; exit ;;
    # TODO: Use this to check if catkin ws already set up
    -d) DEPS_DIR="$2"
        shift 2;;
    -?) echo "error: option -$OPTARG is not implemented"; exit ;;
  esac
done


source $DIR/bash_tools.sh
instlog "Making sure we're in the catkin directory"
# Check if directory is called "src"
if ! ls | grep CMakeLists.txt; then
    instwarn "Could not find a file called CMakeLists.txt"
    instwarn "Need to be in catkin_ws/src, install failed"
    exit
fi
if ! [ $(basename $PWD) == "src" ]; then
    instwarn "Need to be in catkin_ws/src, install failed"
    exit
fi

mkdir -p $DEPS_DIR

instlog "Fixing that stupid chrome thing"
sudo sed -i -e 's/deb http/deb [arch=amd64] http/' "/etc/apt/sources.list.d/google-chrome.list"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -


# Get ready to install ROS
instlog "Getting stuff to install ROS"

instlog "Updating apt-get"
sudo apt-get update -qq

instlog "Getting build stuff"
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial
sudo apt-get install -qq cmake python-pip
sudo pip install -q -U setuptools
sudo pip install -q -U mayavi
sudo pip install -q -U argcomplete
sudo pip install -q -U scikit-learn
sudo pip install -q -U tqdm

####### Make tools
instlog "Getting misc make tools"
sudo apt-get install -qq binutils-dev

instlog "Getting ROS packages we need to install from source"
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/ros-simulation/gazebo_ros_pkgs.git

# Normal things
instlog "Installing misc dependencies"


# sudo apt-get install -qq ros-indigo-gazebo7-ros-pkgs
sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control
sudo apt-get install -qq ros-indigo-control-toolbox ros-indigo-controller-manager ros-indigo-transmission-interface ros-indigo-joint-limits-interface ros-indigo-hardware-interface

sudo apt-get install -qq libboost-all-dev python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq libpcl-1.7-all libpcl-1.7-all-dev ros-indigo-pcl-conversions
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq ros-indigo-sophus
sudo apt-get install -qq ros-indigo-driver-base
sudo apt-get install -qq ros-indigo-camera-info-manager
sudo apt-get install -qq ros-indigo-spacenav-node
sudo apt-get install -qq ros-indigo-camera1394
sudo apt-get install -qq libusb-1.0-0-dev
sudo apt-get install -qq ros-indigo-stereo-image-proc

# catkin_make -C $INSTALL_FOLDER/..
