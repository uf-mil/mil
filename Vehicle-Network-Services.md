The network infrastructure on vehicles is set up to be a self contained system and should rarely need to be changed by anyone other than the network administrator (currently Anthony Olive). In the rare case that something does need to be fixed or updated, this page is here to provide a comprehensive guide for each of the network services that we run.

# Accessing the network

If the network infrastructure is performing it's function properly, users should not have to think about it at all. Connecting to the network should be as simple as connecting an ethernet cable to one of the switches with automatic networking enabled in your OS (also known as DHCP, this should be the default).

# Resolving network problems

In the event that establishing a network connection fails, there are a few things that users can try to fix it. If the network administrator is present, obviously bug them before messing with any configurations. If the vehicle was only recently powered on, be sure to wait a while before jumping to conclusions; it takes a couple minutes to bring up the OS. While you're waiting, double check that all power and network cables are properly connected on the vehicle, switch, and client computer. Unplugged cables are the number one cause of hair loss. Only you can prevent unplugged cables.

If you are unable to get a DHCP lease from the vehicle (i.e. you attempt to connect, but cannot):
* Try setting up a static connection with ip=192.168.37.254 and netmask=255.255.255.0
* If you can now connect to and ping the vehicle, this means there is a problem with DHCP
* Further troubleshooting of the DHCP server should be done through SSH

If you are connected, but hostnames (e.g. mil-sub-sub8.ad.mil.ufl.edu or mil-nav-wamv.ad.mil.ufl.edu) are not responding to connections or pings:
* Try pinging the device's IP address (e.g. 192.168.37.60 or 192.168.37.82)
* If this works, the DNS server is likely not responding to queries
* Further troubleshooting of the DNS server should be done through SSH

If you are connected and can resolve hostnames, but cannot connect with SSH:
* Follow the steps in the block below this one to get a troubleshooting shell
* Someone may have messed with the SSH configuration, so check the `/etc/ssh/sshd_config` file (yes, this has happened before)
* Someone may have messed with the firewall configuration, so see the Firewall and Routing section below

If you are still unable to make any sort of network connection to the vehicle and are bashing your head on the table:
* I wish you luck on this emotional journey, try not to break anything
* Feel free to contact the network administrator to help you troubleshoot, he will grudgingly oblige
* On SubjuGator
    * Try setting up a static connection with ip=192.168.37.254 and netmask=255.255.255.0
    * Connect to the IPMI server running on the vehicle's motherboard in a web browser (mil-sub-ipmi.ad.mil.ufl.edu or 192.168.37.63)
    * Log in to the "subjugator" account
        * Contact the network administrator for the password
    *  Open a console by clicking `Remote Control -> iKVM/HTML5`
    *  This console works similarly to connecting a physical monitor and keyboard, press `Ctrl + Alt + F1` using the virtual keyboard to open a TTY for further troubleshooting
    *  If IPMI is inaccessible, you will likely need to open the main vessel and break out a monitor and keyboard to troubleshoot the problem
* On NaviGator
    * Break out the good old monitor and keyboard (the motherboard purchased for this project does not have IPMI)
    * Press `Ctrl + Alt + F1` to get to a TTY if you are shown a graphical interface

# DHCP Server

The vehicles use a package called isc-dhcp-server to assign DHCP leases. In very simple terms, this enables client computers to obtain an IP address on the network based on the MAC address of the device that they are connecting with. The servers are set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/isc-dhcp-server).

When connected to the MIL local network, vehicles will not activate their own DHCP servers so as to not cause conflicts with IP assignment. In the field, vehicle DHCP servers should automatically activate. The servers are configured to be non-authoritative, meaning other DHCP servers on the network will be able to override them. This is mainly a precaution in case someone accidentally connects the testing switch to another router. This should **never** be done as it will cause a bunch of fun network conflicts for all involved parties.

In MIL, a few different IP ranges have been laid out as follows:
* 192.168.37.1 - 192.168.37.19: Network Infrastructure
* 192.168.37.20 - 192.168.37.29: Physical Servers
* 192.168.37.30 - 192.168.37.49: Virtualized Servers
* 192.168.37.50 - 192.168.37.59: MIL Common
* 192.168.37.60 - 192.168.37.69: SubjuGator
* 192.168.37.70 - 192.168.37.79: PropaGator
* 192.168.37.80 - 192.168.37.89: NaviGator
* 192.168.37.90 - 192.168.37.99: Reserved
* 192.168.37.100 - 192.168.37.119: Reserved Clients
* 192.168.37.120 - 192.168.37.140: Assigned Clients
* 192.168.37.150 - 192.168.37.249: DHCP Pool
* 192.168.37.250 - 192.168.37.255: Reserved

A complete listing of all configured static IP address reservations can be found in the MIL LAN IP Allocation Table file on the MIL fileserver share. This file serves as the master list for hosts in MIL and **any additions to or deletions from the isc-dhcp-server configuration file should be reflected there**. If that file is updated, please inform Daniel Dugger of the change so that he can propagate it to the lab's local network. Any changes should also be reflected in the bind9 DNS server configuration files (see "DNS" below).

IP allocations can be found at the bottom of the file and will look something like this:

    host hostname {
      hardware ethernet XX:XX:XX:XX:XX:XX;
      fixed-address 192.168.37.YY;
    }

* hostname - The hostname for the client machine
* XX:XX:XX:XX:XX:XX - the MAC address of **the connecting interface**, meaning it should be the interface used to connect to the network, not just the default one.
* 192.168.37.YY - the IP that the client will be given; it must be in the pre-defined range of for the device (e.g. a client computer's IP needs to be between 192.168.37.120 and 192.168.37.140).

To find the hostname of a Linux machine, run this command:

    hostname

Please place new entries in the correct commented block based on the type of device they are for and make sure that they are in descending numerical order. If the change is not urgent or you do not understand all of this section, it is highly recommended to have the network administrator make it instead.

# DNS Server

The vehicles use a package called bind9 to resolve local DNS queries. For our local network, the domain that all devices are on is *.ad.mil.ufl.edu. As an example, one could use the hostname client-pc.ad.mil.ufl.edu instead of the IP address 192.168.37.120. The servers are set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/BIND9ServerHowto).

Before adding any new hostnames to this file, **ensure that they are listed as DHCP entries** (see above for details) and are present in the MIL LAN IP Allocation Table file on the MIL fileserver share. This file serves as the master list for hosts in MIL and **any additions to or deletions from the bind9 configuration files should be reflected there**. If that file is updated, please inform Daniel Dugger of the change so that he can propagate it to the lab's local network.

To add a new entry for hostname-based lookup, edit the /etc/bind/db.192 and add a line in ascending numerical order based on the device's IP address:

    ip      IN      PTR     hostname

* ip - The digits after the last decimal place of the client's IP address (e.g. 192.168.37.YY)
* IN - Leave this as is
* PTR - Leave this as is
* hostname - The hostname for the client machine

To add a new entry for reverse lookup (this allows an IP query to resolve to a hostname), edit the /etc/bind/db.ad.mil.ufl.edu and add a line in ascending numerical order based on the device's IP address:

    hostname        IN      A       ip

* hostname - The hostname for the client machine
* IN - Leave this as is
* A - Leave this as is
* ip - The client's full IP address (e.g. 192.168.37.YY)

Make sure that changes made are reflected in both lookup files. **All of the spacing in these two files is done with tabs, not spaces!** If you use spaces in these files, I will find you...

# NTP Server

The vehicles use a package called ntp to keep the clocks on all network computers precisely synchronized. This is not required for client computers that come and go as those will likely be synced with some other internet time server. It is primarily intended for use by the sub and MIL workstations to prevent clock drift. It may also be useful to set a client pc to use the MIL network time server if ROS always throws time synchronization errors on it (this should not be an issue as we normally use simulated time). The servers are set up with some sane defaults, so they should not need to be changed at all. For more information about the package, see [this tutorial](https://help.ubuntu.com/lts/serverguide/NTP.html).

The MIL network time server keeps itself synchronized with the UF network time servers as well as a well established pool of internet network time servers. The following lines present in /etc/ntp.conf enable this:

    # Sync with UF's internal NTP servers
    server ntps2-1.server.ufl.edu iburst
    server ntps2-2.server.ufl.edu iburst
    server ntps2-3.server.ufl.edu iburst

    # Sync with the servers at pool.ntp.org
    server 0.us.pool.ntp.org iburst
    server 1.us.pool.ntp.org iburst
    server 2.us.pool.ntp.org iburst
    server 3.us.pool.ntp.org iburst

The vehicle network time servers keeps themselves synchronized with the MIL network time server. The following lines present in /etc/ntp.conf enable this:

    # Sync with the MIL network ntp server while in the lab
    server mil-svc-1.ad.mil.ufl.edu iburst

In order to use the MIL network time server and the vehicle network time servers, perform the steps listed below. Only use these if you understand what they do because they are not necessary under normal circumstances.

Running this command will force-correct your system clock based on the MIL network's time:

    sudo apt-get install ntpdate
    ntpdate mil-svc-1.ad.mil.ufl.edu

Install the ntp daemon:

    sudo apt-get install ntp

Add this to the servers section of the /etc/ntp.conf file:

    # Sync with the MIL network ntp server while in the lab
    server mil-svc-1.ad.mil.ufl.edu iburst

    # Sync with the vehicle ntp servers while testing in the field
    server mil-sub-sub8.ad.mil.ufl.edu iburst
    server mil-nav-wamv.ad.mil.ufl.edu iburst

Restart the ntp daemon:

    sudo service ntp restart

# Firewall and Routing

The server uses a package called ufw to handle packet routing and firewall functionality. This package provides a simple interface to the iptables packet routing system in the Linux kernel. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/UFW). This is the output of `ufw status verbose` on one of the vehicles:

    Status: active
    Logging: on (low)
    Default: deny (incoming), allow (outgoing)
    New profiles: skip

    To                         Action      From
    --                         ------      ----
    22/tcp                     ALLOW IN    Anywhere
    Anywhere                   ALLOW IN    192.168.37.0/24
    22/tcp                     ALLOW IN    Anywhere (v6)

This configuration defaults to blocking incoming connections and allowing outgoing connections. SSH on port 22 is allowed in from any IP address so that users will always be able to access it for troubleshooting. Incoming traffic is allowed from any 192.168.37.* IP address to any port. This is due to the fact that ROS requires many ports to be open and we trust the clients of this network on vehicles.

As a side note, it is highly recommended that users configure firewalls on their own Linux machines as the default is to have all ports open on any network.