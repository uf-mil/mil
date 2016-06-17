Sub switch is a Cisco WS-2960G-8TC-L, running IOS 12.2(50)SE5

Sub switch is **not to be plugged into the MIL network, IP conflicts will occur as well as competing DHCP servers**

SSH into 192.168.1.1, port 22. Login is sub/sub. Enable secret is sub.

--

Adding a host to DHCP:

SSH into the switch at 192.168.1.5, username sub, password sub. Type en and press enter to enter enable mode - enable secret is sub.

Type "conf t" to get into configuration mode.

Enter "ip dhcp pool **hostname**"

Enter "host 192.168.1.XXX 255.255.255.0"

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