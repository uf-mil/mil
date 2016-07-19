Because it may sometimes be impossible or impractical to work in the lab, a few methods have been set up so that services on the MIL local network can be accessed remotely. If anything else besides the methods listed below is required, just ask Anthony to add it to the list. It should be easy to do so long as the protocol is secured with strong encryption and authentication.

# Lore

Originally, MIL was going to be given a block of static IP addresses from UF; however, this turned into the battle of the ages. Several months ago, a support ticket was put in by Daniel that ended up revealing the level of bureaucratic hell we were dealing with in the IT department. In very dark days, the threat of UFIT using weapons of mass destruction (copious amounts of paperwork) on the lab, a VPN tunnel was set up to end the madness. Currently, the server establishes a connection to Anthony's network (iris-systems.net). The VPN server is able to forward connections from the mil subdomain to the local mil network. Long story short, we have remote access... for now...

# Accessing Bag Files

Because SSHFS is a secure protocol and Samba is inherently not, SSHFS is the only way to access bag files remotely. SSHFS is well documented in [this article](https://github.com/uf-mil/Sub8/wiki/Accessing-Bag-Files), so the command structure will not be covered again here.

In order to use SSHFS remotely, the user must have transfered a copy of their SSH key fingerprint to the server. This is due to the fact that passwords in the lab are simple and not very secure. For more information on this process, see [this article](https://github.com/uf-mil/Sub8/wiki/SSH-Keys).

This example has been taken from the local bag access tutorial and modified for a remote connection:

    sshfs ros-bags@mil.iris-systems.net: /your/local/bags/directory -p 4242 -o reconnect

The ros-bags user can be changed to whatever user is to be authenticated with, but the hostname and port must remain as is.

**IMPORTANT:** once the connection is established, a message similar to the one below will print to the terminal:

    The authenticity of host '[mil.iris-systems.net]:4242 ([50.162.215.108]:4242)' can't be established.
    ED25519 key fingerprint is 69:89:8a:8d:e3:ac:07:71:6f:0a:db:25:77:1c:53:a7.
    Are you sure you want to continue connecting (yes/no)?

This is a security measure built in to the SSH protocol to prevent man in the middle attacks and the key fingerprint should match one of the one's listed below based on the key type being used:
* ED25519 - 69:89:8a:8d:e3:ac:07:71:6f:0a:db:25:77:1c:53:a7
* RSA - 1c:8d:7d:83:71:e4:eb:89:4e:78:a6:be:81:ae:77:42
If the fingerprint does not match one of these, it is **strongly** advised to disconnect (i.e. type **no**)

# Connecting to mil-johnny-five Over SSH

This workstation may be better recognized as the 'Gazebo' or 'Cuda' machine. It can be used for tasks that require high performance simulations or processing large amounts of data. The process to access it remotely is as simple as any other SSH connection.

In order to use SSH remotely, the user must have transfered a copy of their SSH key fingerprint to the server. This is due to the fact that passwords in the lab are simple and not very secure. For more information on this process, see [this article](https://github.com/uf-mil/Sub8/wiki/SSH-Keys).

An example command is shown here:

    ssh uf-mil@mil.iris-systems.net -p 6969

The uf-mil user can be changed to whatever user is to be authenticated with, but the hostname and port must remain as is.

**IMPORTANT:** once the connection is established, a message similar to the one below will print to the terminal:

    The authenticity of host '[mil.iris-systems.net]:6969 ([50.162.215.108]:6969)' can't be established.
    ED25519 key fingerprint is 7d:e5:19:0f:27:b0:b4:e5:cf:59:8b:17:8b:04:da:73.
    Are you sure you want to continue connecting (yes/no)?

This is a security measure built in to the SSH protocol to prevent man in the middle attacks and the key fingerprint should match one of the one's listed below based on the key type being used:
* ED25519 - 7d:e5:19:0f:27:b0:b4:e5:cf:59:8b:17:8b:04:da:73
* RSA - 69:20:65:f9:7a:e8:66:0b:98:33:db:01:b7:a0:9c:fc
If the fingerprint does not match one of these, it is **strongly** advised to disconnect (i.e. type **no**)

# Connecting Over a VPN Tunnel

It has been determined that ROS's lack of consistency in selecting ports to send data over, a VPN is necessary to develop with a connection to any remote ROS core. This could be necessary if the user needs to access the sub or mil-johnny-five computer remotely. The only way to deal with this traffic in a secure and controlled manner is to encapsulate it in a VPN tunnel. The system is being set up to work off of the forwarding VPN infrastructure that is already set up and should be available sometime this week.