MIL Network
###########

MIL uses a standardized network infrastruction that is available
in MIL's rooms and is duplicated for field testing by the :doc:`network box<network_box>`.

LAN
***
The MIL Lan uses subnet ``192.168.37.0`` with mask ``255.255.255.0``. In other words,
all local devices on MIL's network will have an ipv4 address in the form ``192.168.37.x``.

Allocation Table
================

The following is a list of expected static IP addresses on MIL networks:

+------------------------+--------------------------+
| IP                     | Name                     |
+========================+==========================+
| **Network Infrastructure (19)**                   |
+------------------------+--------------------------+
| 192.168.37.1           | Router                   |
+------------------------+--------------------------+
| 192.168.37.19          |                          |
+------------------------+--------------------------+
| **Physical Servers (9)**                          |
+------------------------+--------------------------+
| 192.168.37.20          | esxi-1                   |
+------------------------+--------------------------+
| 192.168.37.21          | idrac-esxi-1             |
+------------------------+--------------------------+
| 192.168.37.29          |                          |
+------------------------+--------------------------+
| **Virtualized Servers (19)**                      |
+------------------------+--------------------------+
| 192.168.37.30          | balin                    |
+------------------------+--------------------------+
| 192.168.37.31          | dc-1                     |
+------------------------+--------------------------+
| 192.168.37.32          | dc-2                     |
+------------------------+--------------------------+
| 192.168.37.33          | fs-1                     |
+------------------------+--------------------------+
| 192.168.37.34          | web                      |
+------------------------+--------------------------+
| 192.168.37.35          | pwd                      |
+------------------------+--------------------------+
| 192.168.37.36          |                          |
+------------------------+--------------------------+
| 192.168.37.37          | acs                      |
+------------------------+--------------------------+
| 192.168.37.38          | pdm                      |
+------------------------+--------------------------+
| 192.168.37.39          | av                       |
+------------------------+--------------------------+
| 192.168.37.40          | svn                      |
+------------------------+--------------------------+
| 192.168.37.41          |                          |
+------------------------+--------------------------+
| 192.168.37.42          | app-1                    |
+------------------------+--------------------------+
| 192.168.37.49          |                          |
+------------------------+--------------------------+
| **MIL Common (9)**                                |
+------------------------+--------------------------+
| 192.168.37.50          | com-gateway              |
+------------------------+--------------------------+
| 192.168.37.51          | com-velodyne-vlp16       |
+------------------------+--------------------------+
| 192.168.37.52          | com-sick-lms111          |
+------------------------+--------------------------+
| 192.168.37.53          | com-teledyne-p900        |
+------------------------+--------------------------+
| 192.168.37.59          |                          |
+------------------------+--------------------------+
| **SubjuGator (9)**                                |
+------------------------+--------------------------+
| 192.168.37.60          | sub8                     |
+------------------------+--------------------------+
| 192.168.37.61          | navtube (RPi)            |
+------------------------+--------------------------+
| 192.168.37.62          | downcam                  |
+------------------------+--------------------------+
| 192.168.37.63          | ipmi                     |
+------------------------+--------------------------+
| 192.168.37.69          |                          |
+------------------------+--------------------------+
| **Reserved (9)**                                  |
+------------------------+--------------------------+
| 192.168.37.70          |                          |
+------------------------+--------------------------+
| 192.168.37.79          |                          |
+------------------------+--------------------------+
| **NaviGator (9)**                                 |
+------------------------+--------------------------+
| 192.168.37.80          | nav-ubnt-shore           |
+------------------------+--------------------------+
| 192.168.37.81          | nav-ubnt-wamv            |
+------------------------+--------------------------+
| 192.168.37.82          | nav-wamv                 |
+------------------------+--------------------------+
| 192.168.37.89          |                          |
+------------------------+--------------------------+
| **Reserved (9)**                                  |
+------------------------+--------------------------+
| 192.168.37.90          |                          |
+------------------------+--------------------------+
| 192.168.37.95          | waterlinked (sub8 DVL)   |
+------------------------+--------------------------+
| 192.168.37.99          |                          |
+------------------------+--------------------------+
| **Reserved Clients (19)**                         |
+------------------------+--------------------------+
| 192.168.37.100         | cad-1                    |
+------------------------+--------------------------+
| 192.168.37.101         | cad-2                    |
+------------------------+--------------------------+
| 192.168.37.102         | shuttle                  |
+------------------------+--------------------------+
| 192.168.37.103         | johnny-five              |
+------------------------+--------------------------+
| 192.168.37.119         |                          |
+------------------------+--------------------------+
| **Assigned Clients (29)**                         |
+------------------------+--------------------------+
| 192.168.37.120         | WLC0002                  |
+------------------------+--------------------------+
| 192.168.37.121         | daniel                   |
+------------------------+--------------------------+
| 192.168.37.125         | keith                    |
+------------------------+--------------------------+
| 192.168.37.137         | cbrxyz (cameron)         |
+------------------------+--------------------------+
| 192.168.37.140         |                          |
+------------------------+--------------------------+
| **DHCP Pool (99)**                                |
+------------------------+--------------------------+
| 192.168.37.150         | DHCP Pool Start          |
+------------------------+--------------------------+
| 192.168.37.249         | DHCP Pool End            |
+------------------------+--------------------------+
| **Reserved (4)**                                  |
+------------------------+--------------------------+
| 192.168.37.250         |                          |
+------------------------+--------------------------+
| 192.168.37.253         |                          |
+------------------------+--------------------------+
| 192.168.37.254         | new pfsense vm           |
+------------------------+--------------------------+

DHCP
****
The MIL network has a DHCP server which will assign a local IP to connected
devices automatically. Some important devices, such as servers and vehicles, have
a static IP address. Other devices are given the next unused IP from the pool
within ``192.168.37.150`` to ``192.168.37.249``.

Connect to Robot / other ROS networks
*************************************
ROS can be configured to talk only on your local machine, or allow communication
from your machine to another, such as one of our robots or a simulation server.

This is done using `ROS environment variables <http://wiki.ros.org/ROS/NetworkSetup>`_.

Assuming you have followed the :doc:`Getting Started Guide</software/getting_started>`,
you can use the function ``ros_connect`` to set this up.

Run ``ros_connect -h`` for an up to date list of options.
For example, run ``ros_connect sub`` to setup this terminal (and children terminals,
such as a ``tmux`` session) to allow your ros nodes, such as ``rviz``, to talk to
the ros nodes on SubjuGator.
