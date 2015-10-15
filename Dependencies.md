To run the Sub, you must install many dependencies. Here is a nice cheat sheet, copied from our Semaphore setup-build script.

This assumes you have installed ros-desktop-full and all the catkin nonsense already.

```
sudo apt-get update -qq
sudo apt-get install -qq libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial

mkdir -p ~/repos
cd ~/repos
git clone https://github.com/vispy/vispy.git
cd vispy
git checkout b48e4d3cf410b853a74b666c475c603e46725e55
sudo python setup.py develop

sudo apt-get install python-ode
rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb


```