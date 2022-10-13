# Networking with Navigator

Connecting to NaviGator is typically done by connecting a base station on the shore
to an access point on the boat. Furthermore, the network box is used by end users
to connect to the base station on the shore. This setup allows us to have a reliable
connection to NaviGator without needing to extend a _very_ long Ethernet cable to
NaviGator.

## Architecture
:::{graphviz} networking.dot
:::

## Network Box

The network box comes with a suite of open ports: some for networking, others for
power. The back of NaviGator has ports for receiving power, while the top of
NaviGator has Ethernet ports for transmitting data to clients over Ethernet. One
of these switches is labelled POE (the middle connector in the top row, near the
red light). This connector should only be used for POE connections, usually to
power the base station over Ethernet.

Inside the network box, you'll find tools for handling power and connectivity. Notably,
a gigabit switch, POE injector, and power supply. These should rarely be touched.
Usually, configuration is best managed through the HTML network UIs, which are
accessible from special IPs.

:::{warning}
Triggering the reset pin on the POE injector will reset the Rocket AC base station
configuration, likely disrupting your network connection. Before pressing this pin,
ensure that you have a backup of the device configuration.
:::

## Rocket AC Base Station and Antenna
A critical component of the network infrastructure is the Rocket AC base station
and connected antenna. This large standing device is connected to the network
box through POE.

The device should be aimed towards the boat to receive the best connection, although
this does not need to be a perfectly straight line. To configure the Rocket AC
device, you can use a web UI known as airOS, provided by the device. To connect,
set your own IP to `192.168.1.XXX`, where `XXX` is some number that is not 20. Then,
connect to the device through `192.168.1.20`. This also works for the Rocket AC
access point located on the boat.
