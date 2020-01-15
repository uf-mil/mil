# MIL Network
MIL uses a standardized network infrastruction that is available
in MIL's rooms and is duplicated for field testing by the [network box](network_box)

## LAN
The MIL Lan uses subnet `192.168.37.0` with mask `255.255.255.0`. In other words, all local devices on MIL's network will have an ipv4 address in the form `192.168.37.x`.

## DHCP
The MIL network has a DHCP server which will assign a local IP to connected devices automaticaly. Some important devices, such as servers and vehicles, have
a static IP address. Other devices are given the next unused IP from the pool within `192.168.37.150` to `192.168.37.249`.
