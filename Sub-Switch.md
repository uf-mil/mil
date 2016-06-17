Sub switch is a Cisco WS-2960G-8TC-L, running IOS 12.2(50)SE5. It is a 8 port layer 2 managed gigabit switch.

CLI is available through SSH on 192.168.1.1, port 22. Login is sub/subjugator. Enable secret is subjugator.

Port 1 is reserved for the tether. This is because all traffic from port 8 is mirrored on port 8 (SPAN). Do not connect your computer to port 8, unless you know what you are doing. There is a cap on port 8 to further prevent accidental use of that port.

**Sub switch is *not* to be plugged into the MIL network, IP conflicts will occur as well as competing DHCP servers**.

The switch assigns IPs to added hosts via DHCP. Only a IP address and netmask is assigned, not a default gateway or DNS server. At the pool name resolution should be handled by your local hosts file.

At the pool, do not connect to WiFi networks that have the 192.168.1.x network. This **will** conflict with the wired connection, and odd behavior **will** occur. If the WiFi network has a network other than 192.168.1.x it is fine to use the WiFi for internet access (such as using your phone to WiFi tether - still check the network though!).

**If you have problems with the wired network at the pool, first restart your computer and/or VM**.

If you have a stale DHCP lease (for example, after coming back into the lab from the pool, or going from the lab to the pool), in Ubuntu run "sudo dhclient -r" to release the lease, and then "sudo dhclient" to renew the lease.

--

Adding a host to DHCP:

SSH into the switch at 192.168.1.1, username sub, password subjugator. Type en and press enter to enter enable mode - enable secret is subjuagtor.

Type "conf t" to get into configuration mode.

Enter "ip dhcp pool xxxxxx" - xxxxxx is the hostname associated to that MAC address.

Enter "host 192.168.1.xxx 255.255.255.0" - xxx is your assigned IP for that specific host.

Enter "hardware-address xxxx.xxxx.xxxx" **for Ubuntu hosts** OR "client-identifier 01xx.xxxx.xxxx.xx" **for Windows or Mac hosts** - xx.xxxx.xxxx.xx is the MAC address.

Type "exit" to exit DHCP host config.

Type "exit" to exit configuration mode.

Type "write" to write config to memory.

Type "exit" to exit the CLI.

--

SNMP Community (RO): public

Ethernet port traffic OIDs:

&nbsp;outgoing 1.3.6.1.2.1.2.2.1.16.portnumber

&nbsp;incoming 1.3.6.1.2.1.2.2.1.10.portnumber