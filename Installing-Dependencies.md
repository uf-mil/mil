To run the Sub, you must install many dependencies. Here is a nice cheat sheet, copied from our Semaphore setup-build script.

This assumes you have installed ros-desktop-full and all the catkin nonsense already. Make sure you have changed your software settings to include "UNIVERSE" packages, and check the source code box.

```
sudo apt-get update -qq
# OMPL stuff
sudo apt-get install -qq libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial

# Simulation OpenGL interface
mkdir -p ~/repos
cd ~/repos
git clone https://github.com/vispy/vispy.git
cd vispy
git checkout b48e4d3cf410b853a74b666c475c603e46725e55
sudo python setup.py develop

# Simulation physics
sudo apt-get install python-ode
rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb

# 3D mouse
sudo apt-get install -qq ros-indigo-spacenav-node
rosdep install spacenav_node
rosmake spacenav_node

```

If your code depends on something not on this page, it will not be pulled!

# SuperInstall

To install absolutely everything, do this

```
sudo apt-get install -qq python-pip
mkdir -p ~/repos
# Add all the apt nonsense
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe"
sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu trusty main universe" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -qq
sudo apt-get install -qq libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq ros-indigo-desktop python-catkin-pkg python-rosdep
sudo apt-get install -qq python-pyode
rm -fr /tmp/pyode-build
mkdir -p /tmp/pyode-build
cd /tmp/pyode-build
sudo apt-get build-dep -y python-pyode
sudo apt-get remove -y python-pyode
apt-get source --compile python-pyode
sudo dpkg -i python-pyode_*.deb
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
echo $PYTHONPATH
catkin_make -C ~/catkin_ws
mv ~/Sub8 ~/catkin_ws/src
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

# Vispy
cd ~/repos
git clone https://github.com/vispy/vispy.git
cd vispy
sudo python setup.py develop

catkin_make -C ~/catkin_ws
```
