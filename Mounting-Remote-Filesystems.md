SSHFS is the primary way to mount a remote filesystem to a local directory because it is lightweight, fast, secure, and implemented on any machine running SSH (i.e. all of our Ubuntu and Debian hosts). First, SSHFS must be installed on the system by running the following:

    sudo apt-get install sshfs

Once that is done, mounting any directory on a remote machine is as simple as:

    sshfs user@host:remote_dir local_dir -o reconnect [-p port]

* user - The username that you want to access files with
* host - The IP address or domain name of the machine you are connecting to
* remote_dir - The directory that you want to mount on the remote machine (if none is specified, this defaults to the home folder for the user you specified)
* local_dir - The directory on your local machine that the bags are mounted to (this must be an empty folder that you have permissions to access)
* port - The port that the SSH server is running on (this defaults to 22, so it is optional in most cases)

This will mount the remote directory to the local directory you specified, making it seem to the OS that the files in that directory are on your system. Simply put, you can interact with them like regular files on your system.

Once you are done copying or streaming files over the tunnel, it is recommended that you unmount SSHFS share with the following:

    sudo umount /your/local/bags/directory

SSHFS can become a little... unstable if the network connection is lost. While the reconnect option fixes some of the issues, a few still remain. Anthony is working on a script to solve this, but no delivery date can be guaranteed at the moment. For now, you should be able to unmount even the most frozen of sessions with this command:

    sudo umount /your/local/bags/directory -lf

The arguments l and f stand for lazy and force, so this should indicate to you that the command should only be run when things go south.