# MIL Network
MIL uses a standardized network infrastruction that is available
in MIL's rooms and is duplicated for field testing by the [network box](network_box)

## LAN
The MIL Lan uses subnet `192.168.37.0` with mask `255.255.255.0`. In other words, all local devices on MIL's network will have an ipv4 address in the form `192.168.37.x`.

## DHCP
The MIL network has a DHCP server which will assign a local IP to connected devices automaticaly. Some important devices, such as servers and vehicles, have
a static IP address. Other devices are given the next unused IP from the pool within `192.168.37.150` to `192.168.37.249`.

## Connect to Robot / other ROS networks
ROS can be configured to talk only on your local machine, or allow communcation
from your machine to another, such as one of our robots or a simulation server.

This is done using [environment variables](http://wiki.ros.org/ROS/NetworkSetup).

Assuming you have followed the [Development guide](/docs/software/development_guide),
you can use the function `ros_connect` to set this up.

Run `ros_connect -h` for an up to date list of options.
For example, run `ros_connect sub` to setup this terminal (and children terminals,
such as a tmux session) to allow your ros nodes, such as rviz, to talk to
the ros nodes on SubjuGator.
