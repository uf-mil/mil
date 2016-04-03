
sudo apt-get install -qq libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
cd ~; hg clone https://bitbucket.org/osrf/gzweb
source /usr/share/gazebo-7/setup.sh
cd ~/gzweb
hg up gzweb_1.2.0
./deploy.sh -m

