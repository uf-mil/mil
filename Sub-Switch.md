Sub switch is a Cisco WS-2960G-8TC-L, running IOS 12.2(50)SE5

SSH into 192.168.1.254, port 22. Login is sub/sub

Enable secret: sub

--

Enter exec (privileged mode, signified by # after hostname):

enable

OR

en

--

Enter configuration mode:

configure terminal

OR

conf t

--

Set enable secret:

enable secret sub
--

Set hostname:

hostname sub8-sw

--

Set domain suffix:

ip domain-name mil.internal

--

Generate RSA keys (2048 max key size on IOS 12):

crypto key generate rsa

--

Add user:

username sub secret sub

--

Enabling SSH (set hostname, domain, add user, and generate RSA keys first):

line vty 0 4

&nbsp;transport input ssh

&nbsp;login local

--

Disabling telnet:

line vty 0 4

&nbsp;transport input none

--

Require console password:

line con 0

&nbsp;login local