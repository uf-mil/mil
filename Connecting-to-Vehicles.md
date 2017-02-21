Connecting to MIL vehicles and workstations over the network has been greatly simplified with some bash magic. If you would like to see all of the cool aliases we have made or want to add your own, see the `scripts/bash_aliases.sh` file in one of our project repositories.

# SSH
SSH (Secure SHell) is an incredible tool that enables you to open a terminal on a remote machine securely over a network. All of our vehicles and Linux workstations run an SSH server. The command takes the following form:

    ssh {user}@{host} [-p {port}]

* user - The username you want to log in to the remote system with
* host - The IP address or domain name of the machine you are connecting to
* port - The port that the SSH server is running on (this defaults to 22, so it is optional in most cases)

There are many ways to authenticate with the server, but passwords and RSA keys are the most popular. For more information on key-based authentication, see the [[SSH Keys]] page. The initial log in must be done with a password, which you can obtain by asking a senior MIL member for access.

We have implemented SSH aliases for each of the vehicles in their respective repositories. For example, `sshsub` can be used to SSH into SubjuGator and `sshnav` can be used to SSH into NaviGator.

# ROS Connect

**Warning: This section only applies to local shells, not shells that are running on the vehicle and connected to over SSH. Basically, ROS Connect is only necessary when a user wants to run a node on their machine or view data in graphical tools like RQT and RVIZ.**

ROS is a network-based message passing protocol, so naturally it needs some information to properly route packets. To send data to and receive data from ROS on one of the vehicles or workstations, users must connect to them. There are three environment variables that must be set in each terminal to control ROS networking:
* ROS_IP - The IP address of your machine
* ROS_HOSTNAME - The hostname of your machine on the network
* ROS_MASTER_URI - The URI of the roscore to connect to, in the form `http://{hostname}:11311`

The aliases implemented by the ros_connect script set these variables for the user in each new terminal they open and prevent them from needing to remember the hostnames of each MIL vehicle and workstation. ROS Connect is based on persistence by default. It uses a `~/.ros_connect_persistence` file to keep track of the currently selected roscore. This means that when a new terminal is opened on your machine, it will connect to the roscore specified in that file. Below is a quick tutorial on using the aliases.

Connect to any available roscore on the network (this will prompt the user if multiple are available):

    ros_connect

Print the help menu for more information:

    ros_connect -h

Toggle persistence on and off:

    ros_connect -p

Make a one-time connection (do not persist this selection to new shells):

    ros_connect -o

Connect to a specific hostname and skip the ~2 seconds of automatic host discovery

    ros_connect -n {hostname}

Flags can be mixed - make a one-time connection to a specific hostname:

    ros_connect -n {hostname} -o

Disconnect from any remote roscores and connect to the local one:

    ros_disconnect