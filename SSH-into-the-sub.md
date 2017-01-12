Networking with the sub
=======================

# Aliases

Add the following aliases to your ~/.bashrc file (or source the `.sub_aliases` file)

    alias setrosip='export ROS_IP=`hostname -I | cut -f1 -d" "`'
    alias rsub='setrosip;export ROS_MASTER_URI=http://sub8:11311'
    alias unsub='unset ROS_IP; export ROS_MASTER_URI=http://localhost:11311'


Then, whenever you want ROS commands to communicate with the roscore running on the sub:

    rsub

And when you would like to return to a roscore on your local machine, just run:

    unsub

_Don't feel that you have to use these silly names._

# Hosts

Add the following line to your /etc/hosts file

    192.168.37.60    sub8

Right after `127.0.0.1 localhost`

Now you're in business.


# SSH

When connected to the sub network, (Ask someone for the password)

    ssh sub8@sub8

You are now in the sub. Don't hurt anything!

If you get an error like "Could not find route, port 22", check that you are actually connected to the network. Remember, unplugged cables are the leading cause of unplugged cables.