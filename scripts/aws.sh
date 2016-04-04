curl -sL https://deb.nodesource.com/setup | sudo bash -
sudo apt-get install nodejs
sudo apt-get install -qq libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
cd ~; hg clone https://bitbucket.org/osrf/gzweb
source /usr/share/gazebo-5/setup.sh
cd ~/gzweb
hg up gzweb_1.2.0
# ./deploy.sh -m

# Watch out for the jeepie jeekies!