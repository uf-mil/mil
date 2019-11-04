The `sshfs` command is the primary way to mount a remote filesystem to a local directory because it is lightweight, fast, secure, and implemented on any machine running SSH (i.e. all of our Ubuntu and Debian hosts). First, `sshfs` must be installed on the system by running the following:

    sudo apt-get install sshfs

Once that is done, mounting any directory on a remote machine is as simple as:

    sshfs <user>@<host>:<remote_directory> <local_directory> -o reconnect [-p <port>]

* `user` - The username that you want to access files with
* `host` - The IP address or domain name of the machine you are connecting to
* `remote_directory` - The directory that you want to mount on the remote machine (if none is specified, this defaults to the home folder for the user you specified)
* `local_directory` - The directory on your local machine that the bags are mounted to (this must be an empty folder that you have permissions to access)
* `port` - The port that the SSH server is running on (this defaults to 22, so it is optional in most cases)

This will mount the remote directory to the local directory you specified, making it seem to the OS that the files in that directory are on your system. Simply put, you can interact with them like regular files on your system.

Once you are done copying or streaming files over the tunnel, it is recommended that you unmount `sshfs` share with the following:

    sudo umount /your/local/bags/directory

The `sshfs` program can become a little... unstable if the network connection is lost. While the reconnect option fixes some of the issues, a few still remain. Anthony is working on a script to solve this, but no delivery date can be guaranteed at the moment. For now, you should be able to unmount even the most frozen of sessions with this command:

    sudo umount /your/local/bags/directory -lf

The arguments l and f stand for lazy and force, so this should indicate to you that the command should only be run when things go south.


# Automatic Mounting with autofs

If you frequently mount a remote filesystem, having it automatically mounted may improve your workflow. See the [[MIL Fileserver]] page to get install autofs and get it set up for SMB shares. The following assumes you have at least installed `autofs` and configured `/etc/auto.master`. Add the following line to `/etc/auto.mil`:

    <local_name>    -fstype=fuse,nodev,noexec,nosuid,uid=<uid>,gid=<gid>,allow_other        :sshfs\#<user>@<host>\:<remote_directory>

* `local_name` - The name of the directory that will be created for the mount (the name of the remote host is recommended)
* `uid` - The user ID for your local user (this should be 1000 on your personal machine)
* `gid` - The group ID for your local user (this should be 1000 on your personal machine)
* `user` - The username that you want to access files with
* `host` - The IP address or domain name of the machine you are connecting to
* `remote_directory` - The path to the directory on the remote host that you want to mount

Your local root user will be the one running autofs, so credentials will need to be generated that allow it to connect to the server. The easiest way to do this is by going through the [[SSH Keys]] tutorial as root (become root by running `sudo -i` - now you have all the power... don't break anything...). Restart the `autofs` daemon by running:

    sudo service autofs restart

Now the mount should exist in the directory you specified.