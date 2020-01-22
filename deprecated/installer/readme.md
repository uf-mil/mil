# Getting Involved

Information about getting involved in MIL is available on the [mil_common wiki](https://github.com/uf-mil/mil_common/wiki). The home page will introduce you to the various projects that are under development in the lab and provide a link to the getting started guide.

# Setting Up the Development Environment

If you have decided to develop for one of our projects, then you will need to set up a development environment that is compatible with it. All of the projects have many dependencies, but there is a convenient install script in this repository to fetch and install them. The script covers all MIL projects that are currently on Ubuntu 16.04 / Debian 8.7 and ROS Kinetic and will allow you to select which one(s) you would like to set up. If the default install location of the catkin workspace at `~/mil_ws` is not ideal, then a different path can be selected when the script prompts for one.

This repository does not need to be cloned locally unless you want to make changes to it. The following command will fetch and run the script:

    sudo apt-get -qq update && sudo apt-get install -qq curl && bash <(curl -s https://raw.githubusercontent.com/uf-mil/installer/master/install.sh)

The install script is intended to handle *every single thing* that needs to be installed to develop for the robot. If it does not work, something has gone terribly wrong and it needs to be fixed. If you resolve an issue while installing, please fix it in the install script and submit a pull-request with the changes. Otherwise, notify the script's maintainer (currently [Anthony Olive](https://github.com/whispercoros)).

The script will create all of the files and directories it needs within the selected catkin workspace. If the script has previously been run, it will not run initial set up tasks that have already been performed unless they need to be updated. This means that the script will respect workspaces with git repositories already present in them.

## Containerized Environment

If you would like to setup the environment in a container then you have a few options such as Docker, LXC, or systemd-nspawn. The following will provide steps to using [systemd-nspawn](https://wiki.archlinux.org/index.php/systemd-nspawn#Create_a_Debian_or_Ubuntu_environment) ("chroot on steroids").

Assumptions:
* Host is a Linux machine
* Using systemd (as systemd-nspawn is part of the package)

Steps:
* Install `debootstrap` (i.e. `sudo apt install debootstrap` or `sudo pacman -S debootstrap`)
* `cd /var/lib/machines && sudo debootstrap --include=systemd-container --components=main,universe xenial MILContainer http://archive.ubuntu.com/ubuntu/`
* Run container: `sudo systemd-nspawn -D MILContainer`
* Set root password `passwd` and logout `logout`
* Allow local apps to access X `sudo xhost +local:` (may need to install xhost package)
* Run complete container (with binded X11 file): `sudo systemd-nspawn -M xenial -b -D MILContainer --bind-ro=/tmp/.X11-unix` and login
* Add nameserver (persistent after reload) `echo 'echo "nameserver 8.8.8.8" > /etc/resolv.conf' >> /etc/rc.local'`
* Add new user `adduser USERNAME` and give sudo `usermod -aG sudo USERNAME`
* `su USERNAME` and set correct `DISPLAY` variable `export DISPLAY=:0.0` and `echo "export DISPLAY=:0.0" >> .bashrc`
* May need to modify `/etc/hosts` to include your hostname
* Run install script from `Setting Up the Development Environment`
* Each time you want to open up the container run: `sudo systemd-nspawn -M xenial -b -D MILContainer --bind-ro=/tmp/.X11-unix`
* Happy coding!


# Setting Up a User Workspace
On vehicles and workstations, the installation process will have already been completed and the user may not necessarily have root access. In these cases, the user install script should be used to perform all of the setup steps to create a catkin workspace without requiring root access.

This repository does not need to be cloned locally unless you want to make changes to it. The following command will fetch and run the script:

    bash <(curl -s https://raw.githubusercontent.com/uf-mil/installer/master/user_install.sh)
