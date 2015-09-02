Networking with the sub
=======================

# Aliases

Add the following aliases to your ~/.bashrc file

    alias setrosip='export ROS_IP=`hostname -I | cut -f1 -d" "/`'
    alias rsub='setrosip;export ROS_MASTER_URI=http://subjugator:11311'
    alias unsub='unset ROS_IP; export ROS_MASTER_URI=http://localhost:11311'


# Hosts

Add the following line to your /etc/hosts file

    192.168.1.21    subjugator

Right after `127.0.0.1 localhost`

Now you're in business.


# SSH

When connected to the sub network, (Ask someone for the password)

    ssh forrest@subjugator

You are now in the sub. Don't hurt anything!

If you get an error like "Could not find route, port 22", check that you are actually connected to the network. Remember, unplugged cables are the leading cause of unplugged cables.