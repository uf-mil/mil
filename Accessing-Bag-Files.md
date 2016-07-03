# Accessing Bags on the Sub

Since bag files are automatically synchronized with the mil-plumbusi server, they are not all on the sub and are likely to be removed and replaced with newer bags when storage space runs low. The most recent bags will still be kept on the sub and can be accessed either inside a linked directory inside the user's home folder or in their actual location, /opt/bags.

If you want to mount the bag files from the sub on your machine, the SSHFS commands below can be modified to suit this goal. Just change user to your username on the sub, host to sub.mil.lan, and the remote_dir to the directory your bags are in.

# Mounting the Bags With SSHFS

SSHFS is the primary way to access bag files on the server because it is lightweight, fast, secure, and implemented on any machine running SSH (e.g. all of our Ubuntu instances). First, SSHFS must be installed on the system by running the following:

    sudo apt-get install sshfs

Once that is done, mounting any directory on a remote machine is as simple as:

    sshfs **user**@**host**:**remote_dir** **local_dir** -o reconnect [-p **port**]

    * **user** - The username that you want to access files with
    * **host** - The IP address or domain name of the machine you are connecting to
    * **remote_dir** - The directory that you want to mount on the remote machine (if none is specified, this defaults to the home folder for the user you specified)
    * **local_dir** - The directory on your local machine that the bags are mounted to (this must be an empty folder that you have permissions to access)
    * **port** - The port that the SSH server is running on (this defaults to 22, so it is optional in most cases)

This will simply mount the remote directory to the local directory you specified, making it seem to the OS that the bags are in that directory on your system. Simply put, you can interact with them like regular files on your system.

To mount the bags on the mil-plumbusi server while connected to the lab's local network, the command takes the following form:

    sshfs **ros-bags**@**mil-plumbusi.mil.lan**: **/your/local/bags/directory** -o reconnect

You will need to get the password from a senior lab member to access this user.

If you are a guest or new to the team, we recommend that you mount the bags in read-only mode so that nothing important is accidentally deleted:

    sshfs __milguest__@**mil-plumbusi.mil.lan**: **/your/local/bags/directory** -o reconnect

The password for this user is available to anyone and can clearly be seen taped to a wall above the desk of our in-house Boxometrist, Ken Tran. You will be able to recognize Ken's desk by the fact that it appears to be a living organism, growing and changing over time.

Once you are done copying or streaming files over the tunnel, it is recommended that you unmount SSHFS share with the following:

    sudo umount **/your/local/bags/directory**

SSHFS can become a little... unstable if the network connection is lost. While the reconnect option fixes some of the issues, a few still remain. Anthony is working on a script to solve this, but no delivery date can be guaranteed at the moment. For now, you should be able to unmount even the most frozen of sessions with this command:

    sudo umount __/your/local/bags/directory__ -lf

The arguments l and f stand for lazy and force, so this should indicate to you that the command should only be run when things go south.

# Mounting the Bags With Samba

Samba is a primarily Windows based fileserver protocol, but there are clients for Linux and Mac OSX. While it is a bit easier to use, it is a verbose networking protocol and options for securing the channel are limited as far as I can tell. Some people *ahem* prefer it, so I have implemented it for use on the local MIL network. There is a good guide for setting Ubuntu up to use Samba [here](https://help.ubuntu.com/community/Samba/SambaClientGuide). There is a Samba share for each SSHFS share, see below:

* ros-bags
    * username = ros-bags
    * workgroup = MIL
    * password = same as user's password for SSHFS

* ros-bags-ro
    * username = milguest
    * workgroup = MIL
    * password = same as user's password for SSHFS