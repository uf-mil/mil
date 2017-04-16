There is a way to mount MIL fileserver shares using the Nautilus file manager GUI, but this guide will only cover doing so in the terminal because the GUI way is just generally bad for long term use. In order to mount the fileserver, you will need to install the utilities to deal with CIFS (e.g. SMB) mounts:

    sudo apt-get install cifs-utils


# Manual Mounting in the Terminal

With the CIFS utilities installed, mounting can be done with the familiar `mount` command:

    sudo mount -t cifs -o uid=<uid>,gid=<gid>,user=<username> //fs.mil.ufl.edu/<share> <local_directory>

* `uid` - The user ID for your user (this should be 1000 on your personal machine)
* `gid` - The group ID for your user (this should be 1000 on your personal machine)
* `username` - A MIL username, which needs to be obtained by talking to the lab's network administrator (currently [Daniel Dugger](https://github.com/duggerd))
* `share` - The share you want to mount (e.g. <username>, mil, subjugator, navigator, etc.)
* `local_directory` - The directory to mount the share on (this must be an empty directory you have permissions to access)


# Automatic Mounting with autofs

The `autofs` utility will handle cleanly mounting fileserver shares when they are available and being used and unmounting them when they are not. It can be installed with the following command:

    sudo apt-get install autofs

The `/etc/auto.master` file containes, you guessed it, the master configuration for the program. This file will be used to refer to a MIL configuration file since users may want to mount other filesystems with it. Append the following line to the end of the file:

    <local_directory>      /etc/auto.mil   --timeout=200,--ghost

* `local_directory` - The directory where all shares will be mounted (Using `/media/mil` is recommended)

Yes, those are tabs, stop your complaining. The `timeout` value specifies how long to wait since the last use to unmount the shares specified in `auto.mil`. This can be extended if you want, but 200 works fairly seamlessly (keep in mind there is only a small delay to mount the share when you next need it). Now, create the `/etc/auto.mil` and place each share on it's own line in the following format:

    <local_name> -fstype=cifs,nodev,noexec,nosuid,uid=<uid>,gid=<gid>,workgroup=MIL,credentials=<credentials_file>       ://fs.mil.ufl.edu/<share>/

* `local_name` - This is the name of the directory that will be created for the share (the share name is recommended)
* `uid` - The user ID for your user (this should be 1000 on your personal machine)
* `gid` - The group ID for your user (this should be 1000 on your personal machine)
* `credentials_file` - The path to your credentials file
* `share` - The share you want to mount (e.g. <username>, mil, subjugator, navigator, etc.)

Once you have created a credentials file, run this command to restart the service with the new configurations:

    sudo service autofs restart

The shares should now be accessible in the local directory you specified in `/etc/auto.mil`.


# Creating a Credentials File

There is probably a way to use a hashed password to establish a connection, but I was unable to find one. Either way, these steps protect the file from prying eyes on a running system. If any part of your disk is encrypted, the file should go there (bonus points if you have Full Disk Encryption). If not, the safest place to stash it is in `/root/.cifs-credentials/mil`. Create the directory and file like this:

    sudo mkdir /root/.cifs-credentials
    sudo chmod 700 /root/.cifs-credentials
    sudo touch /root/.cifs-credentials/mil
    sudo chmod 600 /root/.cifs-credentials/mil

And place the following configurations in the file:

    username=<username>
    password=<password>

* `username` - A MIL username, which needs to be obtained by talking to the lab's network administrator (currently [Daniel Dugger](https://github.com/duggerd))
* `password` - The password associated with the MIL username


# Remote Access

The following is an excerpt from the MIL archives:

> Originally, MIL was going to be given a block of static IP addresses from UF; however, this turned into the
battle of the ages. Several months ago, a support ticket was put in by Daniel that ended up revealing the level of bureaucratic hell we were dealing with in the IT department. In very dark days, the threat of UFIT using weapons of mass destruction (copious amounts of paperwork) on the lab, a VPN tunnel was set up to end the madness.

A year after that was written, we finally have our block of IP addresses. The fileserver is available on `fs.mil.ufl.edu` from anywhere on the UF campus. If off-campus access is required, the [UF Anyconnect VPN server](https://connect.ufl.edu/it/wiki/Pages/glvpn-anyconnect-install.aspx) can be used to tunnel your traffic into the network.