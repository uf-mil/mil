#!/bin/bash
if [ $# -ge 1 ]; then
    DEPS_DIR="$1"
    CATKIN_DIR="$1"/catkin_ws
elif [ $# -ge 2 ]; then
    DEPS_DIR="$1"
    CATKIN_DIR="$2"
else
    DEPS_DIR=~/repos/sub_dependencies
    CATKIN_DIR=~/repos/catkin_ws
fi

echo "Installing dependencies in $DEPS_DIR"
echo "Generating catkin workspace (If needed) at $CATKIN_DIR"

set -e
# TODO: Force this to be run as root, and manually run the non-root commands as w/e
#if (( $EUID != 0 )); then
#    echo "Error: You are not root. Please run 'install.sh' instead"
#    exit
#fi


mkdir -p "$DEPS_DIR"
cd "$DEPS_DIR"
####### Always Pre-requisites

sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe"
sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu trusty main universe" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -qq

sudo apt-get install -qq cmake

####### Make tools
sudo apt-get install -qq binutils-dev

####### Python stuff
# Deal with pyODE
sudo apt-get install -qq python-pyode
rm -fr /tmp/pyode-build
mkdir -p /tmp/pyode-build
cd /tmp/pyode-build
sudo apt-get build-dep -y python-pyode
sudo apt-get remove -y python-pyode
apt-get source --compile python-pyode
sudo dpkg -i python-pyode_*.deb

# Normal things
sudo apt-get install -qq libboost-all-dev python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial

####### Vispy
# Check if Vispy is installed
cd "$DEPS_DIR"
set +e
pip freeze | grep -i vispy
if [ $? -eq 1 ]; then
    set -e
    echo "INSTALLER: Looks like you don't have Vispy, let's grab it"
    # Let's force ourselves to stay at the latest version
    # TODO: python -c "import vispy; print vispy.__version__" -> Compare to what's available
    # TODO: Uninstall old Vispy if it is out of date
    git clone -q https://github.com/vispy/vispy.git
    cd vispy
    sudo python setup.py develop
else
    echo "INSTALLER: You have Vispy, don't worry, we'll make sure it's up to date"
    # How's that for a hack!? (Figure out location of Vispy)
    cd "$(dirname `python -c  "import vispy; print vispy.__file__"`)/.."
    sudo python setup.py develop --uninstall
    if [ $? -eq 1 ]; then
        echo "INSTALLER: Your Vispy installation is weird, don't install in manually, use the the sub install script"
    fi
    git pull
    sudo python setup.py develop
fi
####### End Vispy

####### Ceres
cd "$DEPS_DIR"
# TODO: Make this better (It might not be installed in /usr/local!)
set +e
ls /usr/local/share/ | grep -i ceres
if [ $? -ne 0 ]; then
    set -e
    echo "INSTALLER: Looks like to don't have Google Ceres, we'll install it"
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
fi

cd "$DEPS_DIR"

# TODO: rm -rf ./ceres-solver-1.11.0
####### End Ceres

####### Install ROS
set +e
if which rosrun; then
    echo "INSTALLER: Nice, you already have ROS. We're lazy, so we didn't check the version. Just promise you have Indigo"
else
    echo "INSTALLER: Looks like ROS is not installed, let's take care of that"
    sudo apt-get install -qq ros-indigo-desktop python-catkin-pkg python-rosdep
    set -e
    source /opt/ros/indigo/setup.bash
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    sudo rosdep init
    rosdep update
fi

####### Install the ROS packages that we use
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq ros-indigo-sophus

####### Check if the sub is set up, and if it isn't, set it up
set +e
roscd sub8_controller
if [ $? -ne 0 ]; then
    set -e
    echo "INSTALLER: Looks like you don't have the sub set up, let's do that"
    # Make our sub stuff
    mkdir -p "$CATKIN_DIR/src"
    cd "$CATKIN_DIR/src"
    catkin_init_workspace
    catkin_make -C "$CATKIN_DIR"
    source "$CATKIN_DIR/devel/setup.bash"
    echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc

    # Check if Sub8 repository already exists (Semaphore does this)
    set +e
    echo "INSTALLER: Looking for Sub8 (Are we in Semaphore?)"

    if  ls ~/ | grep -i Sub8; then
        set -e
        # Here, we assume we're in Semaphore-ci, and should run catkin_make in the actual build thread
        echo "INSTALLER: Found Sub8 in HOME, Assuming we're in Semaphore"
        mv ~/Sub8 "$CATKIN_DIR/src"
    else
        set +e
        ls "$CATKIN_DIR/src" | grep Sub8
        if [ $? -ne 0 ]; then
            set -e
            cd "$CATKIN_DIR/src"
            git clone -q https://github.com/uf-mil/Sub8.git
            cd Sub8
            git remote rename origin upstream
            echo "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
            catkin_make -C "$CATKIN_DIR"
        fi
    fi
fi
