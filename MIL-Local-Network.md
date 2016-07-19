MIL uses a server with the hostname mil-plumbusi [(in honor of the plumbus, plumbusi is the plural of plumbus)](https://www.youtube.com/watch?v=_Y-_13eYwBQ) to manage the Local Area Network (LAN) in the lab. This network provides an easy way for devices to communicate with each other and connect to the internet. The server should rarely need to be accessed as it was designed to maintain it's self. If problems arise or changes need to be made, the information in this article will be helpful.

# Accessing the network

**WARNING: No device running Windows is allowed to connect to this network unless explicitly allowed by one of the network administrators in the lab (Anthony Olive or Daniel Dugger)!**

The network can easily be accessed by clients with a wired (ethernet) or a wireless connection. To connect over ethernet, simply plug the computer into a free port on the unmanaged switch, which is located on the same table as the mil-plumbusi server, or on the Sub8 Router, which is located on the same table as the sub. To connect over WiFi, select either the uf-sub-2ghz or uf-sub-5ghz network (uf-sub-5ghz is recommended if it is available on the machine) and ask a senior lab member for the password.

# Networking Hardware

* The mil-plumbusi server connects to
    * MAE's internal network through the wall, which routes to the internet
    * An unmanaged switch on the same table as the server
        * This connection is through a USB Gigabit ethernet adapter, which needs to be replaced with a PCI card

* The unmanaged switch connects to
    * The mil-plumbusi server
    * The mil-johnny-five workstation
    * The Sub8 Router (a dual band wireless router that acts as a network access point)
        * This cable runs across the room through the ceiling (and looks super professional)
    * There are two open ports for future connections
        * Daniel is using one of these at the time of writing

* The Sub8 Router connects to
    * Sub8 is connected via the tether spool
    * There are three open ports for future connections
        * Ralph and David are using two of these at the time of writing
    * This operates in access point mode, enabling wireless access to the network

# Resolving network problems

Please try these steps before trying to modify any configurations:
* Shut down the server by pressing the power button (even though it says not to)
* Make sure that all of the ethernet and USB connections to the back of it are secure
* Boot the server back up and wait a few minutes
* See if the issue has been fixed

If hostnames (e.g. mil-plumbusi.mil.lan) are not responding to connections or pings:
* Try pinging it's IP (e.g. 192.168.1.1)
* If this works, the DNS server is likely not responding to queries
* Further troubleshooting should be done through SSH at this point

If you are unable to get a DHCP lease from the server (i.e. you attempt to connect, but cannot):
* Try setting up a static connection with ip=192.168.1.99, gateway=192.168.1.1, and netmask=255.255.255.0
* If you can now connect to and ping the server, this means there is a problem with DHCP
* Further troubleshooting should be done through SSH at this point

If you are still unable to make any sort of network connection and are bashing your head on the table:
* Connect a monitor and keyboard to the server and enter Ctrl+Alt+F2 to get to a console
* Log in to the plumbusiadmin account
    * See Anthony for the password
* This account has sudo access and can be used to troubleshoot and fix issues
* I wish you luck on this emotional journey, try not to break anything
* Feel free to call Anthony to help you troubleshoot, he will grudgingly oblige

# SSH Access

If something needs to be changed on the server, the best way to access it is via SSH. Log into the plumbusiadmin account with the following command

    ssh plumbusiadmin@mil-plumbusi.mil.lan

If you are wearing your tinfoil hat, you can have a look at this article for the server's key fingerprint. This should not be required on the LAN unless some nefarious business is going on. See Anthony for the password for this account or to have one made for you.

# DHCP Server

The server uses a package called isc-dhcp-server to assign DHCP leases. In very simple terms, this enables client computers to obtain an IP address on the network based on the MAC address of the device that they are connecting with. The server is set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/isc-dhcp-server).

In MIL, a few different IP ranges have been laid out as follows:
* 192.168.1.1-192.168.1.9: MIL computers and networking hardware
* 192.168.1.10-192.168.1.19: Networked devices in the lab (e.g. printers)
* 192.168.1.20-192.168.1.29: Sub related devices
* 192.168.1.30-192.168.1.89: Software team member computers with static IP allocations
* 192.168.1.90-192.168.1.99: Reserved for special use cases (e.g. client-defined static IP's)
* 192.168.1.100-192.168.1.199: Dynamic IP address allocation for all other devices

A complete listing of all configured static IP address reservations can be found in the [scripts/update_hosts.sh](https://github.com/uf-mil/Sub8/blob/master/scripts/update-hosts.bash) file. This file helps client computers and the sub keep track of networked devices when the DNS server is not available and **any additions to or deletions from the isc-dhcp-server configuration file should be reflected there**. They should also be added to the bind9 DNS server for hostname-based lookups (see DNS below).

Static IP allocations can be found at the bottom of the file and will look something like this:

    host hostname {
      hardware ethernet XX:XX:XX:XX:XX:XX;
      fixed-address 192.168.1.YY;
    }

* hostname - The hostname for the client machine
* XX:XX:XX:XX:XX:XX - the MAC address of **the connecting interface**, meaning it could be the ethernet or WiFi address depending on which is being used.
* 192.168.1.YY - the IP that the client will be given; it must be in the pre-defined range of for the device (e.g. a client computer's IP needs to be between 192.168.1.30 and 192.168.1.89). 

To find this out of a linux machine, run this:

    hostname

Multiple interfaces can be specified for the same client computer. For example, if one wants to connect over ethernet sometimes and WiFi other times, both a *client-pc_ethernet* and a *client-pc_wireless* connection can be created. These will have the different MAC addresses of their respective devices, but share the same IP address. While this is a useful trick, **connecting with both ethernet and WiFi will cause major network jankiness** and users should be careful to avoid this. My personal recommendation is to set a static ethernet IP address and use dynamic allocation for WiFi.

Please place new entries in the correct commented block based on the type of device they are for and make sure that they are in descending numerical order.

# DNS Server

The server uses a package called bind9 to resolve local DNS queries. For our local network, the domain that all devices are on is mil.lan. As an example, one could use the hostname sub.mil.lan instead of the IP address 192.168.1.21. The server is set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/BIND9ServerHowto).

Before adding any new hostnames to this file, **ensure that they are listed as static DHCP entries** (see above for details) and are present in the [scripts/update_hosts.sh](https://github.com/uf-mil/Sub8/blob/master/scripts/update-hosts.bash) file.

To add a new entry for hostname-based lookup, edit the /etc/bind/db.192 and add a line in descending numerical order based on the device's IP address:

    ip      IN      PTR     hostname

* ip - The digits after the last decimal place of the client's IP address (e.g. 192.168.1.IP)
* IN - Leave this as is
* PTR - Leave this as is
* hostname - The hostname for the client machine

To add a new entry for reverse lookup (this allows an IP query to resolve to a hostname), edit the /etc/bind/db.mil.lan and add a line in descending numerical order based on the device's IP address:

    hostname                IN      A       ip

* hostname - The hostname for the client machine
* IN - Leave this as is
* A - Leave this as is
* ip - The client's full IP address (e.g. 192.168.1.YY)

**All of the spacing in these two files is done with tabs, not spaces!** If you use spaces in these files, I will find you...

To find this out of a linux machine, run this:

    hostname

# NTP Server

The server uses a package called ntp to keep the clocks on all network computers precisely synchronized. This is not required for client computers that come and go as those will likely be synced with some other internet time server. It is primarily intended for use by the sub and MIL workstations to prevent clock drift. It may also be useful to set a client pc to use mil-plumbusi a network time server if ROS always throws time synchronization errors on it. The server is set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/lts/serverguide/NTP.html).

The mil-plumbusi server keeps itself synchronized with the UF network time servers. The following lines present in /etc/ntp.conf enable this:

    # Sync with UF's internal NTP servers
    server ntps2-1.server.ufl.edu
    server ntps2-2.server.ufl.edu
    server ntps2-3.server.ufl.edu

In order to use mil-plumbusi as a network time server, perform the steps listed below. Only use these if you understand what they do because they are not necessary under normal circumstances.

Running this command will force-correct your system clock based on the network's time:

    sudo apt-get install ntpdate
    ntpdate mil-plumbusi.mil.lan

Install the ntp daemon:

    sudo apt-get install ntp

Add this to the servers section of the /etc/ntp.conf file:

    # Sync with MIL's internal NTP server
    server mil-plumbusi.mil.lan

Restart the ntp daemon:

    sudo service ntp restart

# Firewall and Routing

The server uses a package called iptables to handle packet routing and firewall functionality. The iptables-persistent package is used to store the configuration of the routing tables in configuration files. As we are only using IPV4 in the lab, the only configuration file of interest is /etc/iptables/rules.v4. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/IptablesHowTo).

Anthony has literally spent hours configuring this part of the server, so please **ask him before making any changes** to it. The iptables syntax is very finicky and it is easy to make a mistake that renders the network unusable.

The iptables configuration currently handles the following:
* Blocking all kinds of nefarious traffic
* Making sure network clients can access important services on mil-plumbusi
* Enabling internet access through IP masquerading (Network Address Traversal, or NAT)
* Forwarding certain remote connections from the VPN tunnel to local services (see this article for more information)

# Unattended Upgrades

Being the self-maintaining headless server that it is, mil-plumbusi has a built in service to automatically keep all of it's packages up to date, remove obsolete packages, and reboot when updates require it. This is facilitated by the unattended-upgrades package. Important files to note are /etc/apt/apt.conf.d/10periodic, which controls when and how upgrades are run, and /etc/apt/apt.conf.d/50unattended-upgrades, which controls which packages are upgraded and what to do after upgrading.

The only thing that I can see being changed here is if a new repository has been added to /etc/apt/sources.list (or in /etc/apt/sources.list.d). This should really never happen unless a service that is not in the main Ubuntu repositories is needed. As an important side note, **this server is not meant to run ros, gazebo, or any MIL code**. It is only meant to be reliable network infrastructure.

The following configuration in /etc/apt/apt.conf.d/50unattended-upgrades is used to select which repositories' packages are automatically upgraded:

    // Automatically upgrade packages from these (origin:archive) pairs
    Unattended-Upgrade::Allowed-Origins {
            "${distro_id}:${distro_codename}";
            "${distro_id}:${distro_codename}-security";
            "${distro_id}:${distro_codename}-updates";
            "${distro_id}:${distro_codename}-backports";
            "webudp8:precise";
    };

To add a repository, simply add it's origin and archive names to the list as shown in the examples above. Two aliases are supported by the file:
* ${distro_id} - Will be replaced with ubuntu on this machine
* ${distro_codename} - Will be replaced with trusty on this machine

**All of the spacing in these two files is done with tabs, not spaces!** If you you spaces in these files, I will find you...

# Fail2ban

Since this machine is WAN facing (i.e. it is directly accessible from the internet, albeit through a VPN), I have decided to implement a basic defensive measure by installing the fail2ban package. The fail2ban daemon looks through the system logs for all configured services and if enough occurrences of the configured error message are found (e.g. SSH authentication failures), the connecting IP address is blocked for a limited time period. It is very simple, but effective against most botnets.

If you are the unlucky IP that has been blocked, rebooting the mil-plumbusi server should fix the problem. This is due to the fact that fail2ban uses iptables to manage blocked IP addresses. Since iptables is reloaded from the persistent configuration on startup, all banned IP addresses will be forgotten.

There are many services implemented by the default fail2ban package and even more can be configured by hand if one desires (see [their website](http://www.fail2ban.org/wiki/index.php/Main_Page) for more information).

Our configuration files can be found at /etc/fail2ban/fail2ban.conf and /etc/fail2ban/jail.conf. Here is a list of services that are currently protected:
* pam authentication daemon
* xinetd connection daemon
* ssh
* apache
* named (in our case, bind9)

# Fileserver

There is a 1 TB EXT4 filesystem on the server's /dev/sda drive. It is automatically mounted to /media/mil-data on bootup by fstab. This is the location where all files should be stored for the fileservers. At the time of writing, this only applies to ros bag files for the sub and machine learning data from training, but that will likely change.

There are two different methods configured for accessing the files on the server - SSHFS and Samba. If you wish to set up a new fileserver share, ask Anthony before diving in. He can do this in a couple minutes, but it might be fairly involved if you are not familiar with the process. In any case, the steps below will give you an idea as to how shares are managed as well as take you through the process of creating a new share.

Create a share folder under /media/mil-data as root:
    sudo mkdir /media/mil-data/share

* share - The name of the share being created

For ease of implementation, a user is created for each access group of each share. New users should be created with the following commands:

    sudo useradd -m -s /usr/sbin/nologin username
    passwd username

* username - The desired username for logging in to the share

The actual home directory for the user is deleted and replaced by a bind mount to the files that that user should have access to. This command deletes the automatically created home directory files:

    sudo rm -rf /home/username/*

* username - The username for logging in to the share

A line should be added for the share in /etc/rc.local inside of the block beginning with the comment "Link filesystem users to their data". If the user should have read and write access to the share, use this line:

    bindfs -u UID -g GID --create-for-user=root --create-for-group=root /media/mil-data/share /home/username

* UID - The id of the user you just created
* GID - The id of the new user's group
* share - The folder that the share is stored under on the fileserver drive
* username - The username for logging in to the share

For read-only access to the share, use this line instead:

    bindfs -u UID -g GID -r /media/mil-data/share /home/username

* UID - The id of the user you just created
* GID - The id of the new user's group
* share - The folder that the share is stored under on the fileserver drive
* username - The username for logging in to the share

To make the user accessible over ssh, edit the /etc/ssh/sshd_config file and find a line that begins with the AllowUsers directive. Append the name of your new user to the line, which should have the following format:

    AllowUsers user1 user2 user3

This is all that is required to create a new SSHFS share. After rebooting the server, you should be able to use sshfs, as documented [here](https://github.com/uf-mil/Sub8/wiki/Accessing-Bag-Files), to connect to the share with a password. You should also be able to copy your key file over to the server for ease of access, as documented [here](https://github.com/uf-mil/Sub8/wiki/SSH-Keys).

Enabling samba access to the share is easy and it is highly recommended that each share be accessible over both fileservers unless a specific circumstance warrent that they not be. A configuration block for the new share needs to be added to the bottom of the /etc/samba/smb.conf file. The format for this is as follows:

    [share]
       comment = Example description of a share
       browsable = no
       path = /home/username
       guest ok = no
       read only = no
       create mask = 0700
       directory mask = 0700
       valid users = username

* share - The name of your new share
* comment - A brief description of what the share contains
* browsable - Whether or not samba clients can see this share in a scan
* path - The path to the access user's home directory
* guest ok - Whether or not anonymous (unauthenticated) users can access the share
* read only - Determines if the share is read only or writable
* create mask - Leave as is
* directory mask - Leave as is
* valid users - Users that have access to the share (i.e. the new access user you created)

After that file is saved, a password entry must be created for the user in the samba authentication database:

    smbpasswd -a username

* username - The username for logging in to the share

It is recommended to set the samba password to the same thing as the system password or else there will be unnecessary confusion. After that, restart the samba daemon:

    service samba restart

The share should now be accessible over both SSHFS and Samba. This should also work remotely over SSHFS as detailed here. Enjoy your free large file storage!