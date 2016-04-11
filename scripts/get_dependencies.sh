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


# Get ready to install ROS
instlog "Getting stuff to install ROS"

instlog "Updating apt-get"
sudo apt-get update -qq

instlog "Getting build stuff"
sudo apt-get install -qq cmake python-pip
sudo pip install -q -U setuptools
sudo pip install -q -U mayavi
sudo pip install -q -U argcomplete

####### Make tools
instlog "Getting misc make tools"
sudo apt-get install -qq binutils-dev

instlog "Getting python packages we need to install from source"
python_from_git https://github.com/vispy/vispy.git vispy vispy 0495d8face28571ad19c64cbc047327b084a7c03

instlog "Getting ROS packages we need to install from source"
ros_git_get https://github.com/txros/txros.git
ros_git_get https://github.com/uf-mil/rawgps-tools.git
ros_git_get https://github.com/ros-simulation/gazebo_ros_pkgs.git

# Ceres
cd "$DEPS_DIR"
# TODO: Make this better (It might not be installed in /usr/local!)
if ! ls /usr/local/share/ | grep --quiet -i ceres$; then
    instlog "Looks like to don't have Google Ceres, we'll install it"
    sudo apt-get -qq install libgoogle-glog-dev
    # BLAS & LAPACK
    sudo apt-get -qq install libatlas-base-dev
    # Eigen3
    sudo apt-get -qq install libeigen3-dev
    # SuiteSparse and CXSparse (optional)
    # - If you want to build Ceres as a *static* library (the default)
    #   you can use the SuiteSparse package in the main Ubuntu package
    #   repository:
    sudo apt-get -qq install libsuitesparse-dev
    wget http://ceres-solver.org/ceres-solver-1.11.0.tar.gz
    # Unzip
    tar zxf ceres-solver-1.11.0.tar.gz
    # Delete the zip trash
    rm ./ceres-solver-1.11.0.tar.gz
    mkdir ceres-bin
    cd ceres-bin
    cmake ../ceres-solver-1.11.0
    make -j3
    sudo make install
else
    instlog "Looks like you already have ceres"
fi

instlog "Checking if we need to fix pyode"
if ! python -c "import ode; w = ode.World(); w.setAngularDamping(0.2)"; then
    instlog "Fixing/installing Pyode"
    sudo apt-get install -qq python-pyode
    sudo rm -fr /tmp/pyode-build
    sudo mkdir -p /tmp/pyode-build
    cd /tmp/pyode-build
    sudo apt-get build-dep -qq -y python-pyode
    sudo apt-get remove -qq -y python-pyode
    sudo apt-get source -qq --compile python-pyode
    sudo dpkg -i python-pyode_*.deb 2>&1 >/dev/null
    sudo apt-mark hold pyode

else
    instlog "We don't need to fix pyode! How lucky!"
fi


# Normal things
instlog "Installing misc dependencies"

sudo apt-get install -qq ros-indigo-gazebo7-msgs ros-indigo-gazebo7-ros ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros-control
sudo apt-get install -qq ros-indigo-control-toolbox ros-indigo-controller-manager ros-indigo-transmission-interface ros-indigo-joint-limits-interface ros-indigo-hardware-interface

sudo apt-get install -qq libboost-all-dev python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial
sudo apt-get install -qq libpcl-1.7-all libpcl-1.7-all-dev ros-indigo-pcl-conversions
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq ros-indigo-sophus
sudo apt-get install -qq ros-indigo-driver-base
sudo apt-get install -qq ros-indigo-camera-info-manager
sudo apt-get install -qq ros-indigo-spacenav-node
sudo apt-get install -qq libusb-1.0-0-dev

# catkin_make -C $INSTALL_FOLDER/..
