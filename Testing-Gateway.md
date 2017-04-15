The testing gateway is a Raspberry PI that allows us to connect the vehicle networks to networks that have access to the Internet. This is extremely useful for testing days as it allows us to look up information, pull packages, push code to Github, and even access MIL services if the network we are connecting to is run by UF.


# Connecting to the Local Network

The gateway box connects to the lab or vehicle network using a static IP so that it is not dependent on that vehicle's DHCP server. This enables connections to and from clients on that network. The configuration for the onboard ethernet interface is in the `/etc/dhcpcd.conf` file. Below is the excerpt that is important:

    interface eth0
            static ip_address=192.168.37.50
            static domain_name_servers=192.168.37.31 192.168.37.32 192.168.37.60 192.168.37.82
            static domain_name=ad.mil.ufl.edu

There are a couple important notes about the nameservers used here. The `192.168.37.31` and `192.168.37.32` servers are the internal ones for MIL. The `192.168.37.60` and `192.168.37.82` entries are the IP addresses of SubjuGator and NaviGator. This enables the gateway box to resolve MIL hosts both on the MIL network and while at a testing site; however, the primary nameserver is defined by the DHCP connection to the external network, allowing host lookups on the Internet.


# Connecting to an External Network

By default, the Raspberry Pi is configured to use the built in WiFi card to connect to the `uf` SSID; however, it can easily be adapted to another wireless network or even a wired network with a USB to ethernet connector. The following is an excerpt from the `/etc/network/interfaces` file:

    allow-hotplug wlan0
    iface wlan0 inet manual
            wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf

This is the `/etc/network/interfaces` file:

    country=US
    ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
    update_config=1

    network={
            ssid="uf"
            key_mgmt=WPA-EAP IEEE8021X
            eap=PEAP
            auth_alg=OPEN
            phase1="peaplabel=0"
            phase2="auth=MSCHAPV2"
            identity="<gatorlink_username>"
            password="<gatorlink_password>"
            ca_cert="/etc/wpa_supplicant/uf-peap-ca.cer"
    }

This is the `/etc/wpa_supplicant/uf-peap-ca.cer` file:

    -----BEGIN CERTIFICATE-----
    MIIENjCCAx6gAwIBAgIBATANBgkqhkiG9w0BAQUFADBvMQswCQYDVQQGEwJTRTEU
    MBIGA1UEChMLQWRkVHJ1c3QgQUIxJjAkBgNVBAsTHUFkZFRydXN0IEV4dGVybmFs
    IFRUUCBOZXR3b3JrMSIwIAYDVQQDExlBZGRUcnVzdCBFeHRlcm5hbCBDQSBSb290
    MB4XDTAwMDUzMDEwNDgzOFoXDTIwMDUzMDEwNDgzOFowbzELMAkGA1UEBhMCU0Ux
    FDASBgNVBAoTC0FkZFRydXN0IEFCMSYwJAYDVQQLEx1BZGRUcnVzdCBFeHRlcm5h
    bCBUVFAgTmV0d29yazEiMCAGA1UEAxMZQWRkVHJ1c3QgRXh0ZXJuYWwgQ0EgUm9v
    dDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALf3GjPm8gAELTngTlvt
    H7xsD821+iO2zt6bETOXpClMfZOfvUq8k+0DGuOPz+VtUFrWlymUWoCwSXrbLpX9
    uMq/NzgtHj6RQa1wVsfwTz/oMp50ysiQVOnGXw94nZpAPA6sYapeFI+eh6FqUNzX
    mk6vBbOmcZSccbNQYArHE504B4YCqOmoaSYYkKtMsE8jqzpPhNjfzp/haW+710LX
    a0Tkx63ubUFfclpxCDezeWWkWaCUN/cALw3CknLa0Dhy2xSoRcRdKn23tNbE7qzN
    E0S3ySvdQwAl+mG5aWpYIxG3pzOPVnVZ9c0p10a3CitlttNCbxWyuHv77+ldU9U0
    WicCAwEAAaOB3DCB2TAdBgNVHQ4EFgQUrb2YejS0Jvf6xCZU7wO94CTLVBowCwYD
    VR0PBAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wgZkGA1UdIwSBkTCBjoAUrb2YejS0
    Jvf6xCZU7wO94CTLVBqhc6RxMG8xCzAJBgNVBAYTAlNFMRQwEgYDVQQKEwtBZGRU
    cnVzdCBBQjEmMCQGA1UECxMdQWRkVHJ1c3QgRXh0ZXJuYWwgVFRQIE5ldHdvcmsx
    IjAgBgNVBAMTGUFkZFRydXN0IEV4dGVybmFsIENBIFJvb3SCAQEwDQYJKoZIhvcN
    AQEFBQADggEBALCb4IUlwtYj4g+WBpKdQZic2YR5gdkeWxQHIzZlj7DYd7usQWxH
    YINRsPkyPef89iYTx4AWpb9a/IfPeHmJIZriTAcKhjW88t5RxNKWt9x+Tu5w/Rw5
    6wwCURQtjr0W4MHfRnXnJK3s9EK0hZNwEGe6nQY1ShjTK3rMUUKhemPR5ruhxSvC
    Nr4TDea9Y355e6cJDUCrat2PisP29owaQgVR1EX1n6diIWgVIEM8med8vSTYqZEX
    c4g/VhsxOBi0cQ+azcgOno4uG+GMmIPLHzHxREzGBHNJdmAPx/i9F4BrLunMTA5a
    mnkPIAou1Z5jJh5VkpTYghdae9C8x49OhgQ=
    -----END CERTIFICATE-----

UF's network uses WPA Enterprise, so wpa_supplicant must be used to configure additional settings for it. The PEAP certificate is used for validation of the connection and Gatorlink user credentials must be included for authentication. For this to work with a network running WPA Personal (i.e. most WiFi networks), change the `/etc/network/interfaces` excerpt:

    allow-hotplug wlan0
    iface wlan0 inet dhcp
            wpa-ssid <network_ssid>
            wpa-psk <network_passphrase>

Here, the network name (SSID) and passphrase are included in the file for authentication. Finally, for this to work with a wired network, change the `/etc/network/interfaces` excerpt:

    allow-hotplug eth1
    iface eth1 inet dhcp

This configuration assumes the USB to ethernet connector is registered as `eth1` because `eth0` is the onboard ethernet interface. This can be different, so be sure to check by running `ifconfig -a`.


# Bridging the Networks with NAT

The testing gateway uses a package called iptables to handle packet routing and firewall functionality. For more information about the package, see [this tutorial](https://help.ubuntu.com/community/IptablesHowTo). The routing table is loaded on boot by the following line in `/etc/rc.local`:

    sh /etc/iptables-routing.sh

This calls the iptables setup script at `/etc/iptables-routing.sh`, which contained the following stable configuration at the time of writing:

    #!/bin/sh

    # External interface
    EXTIF="wlan0"

    # External IP address (automatically detected)
    EXTIP=$(/sbin/ip addr show dev "$EXTIF" | perl -lne 'if(/inet (\S+)/){print$1;last}');

    # Internal interface
    INTIF="eth0"

    # Internal IP address (in CIDR notation)
    INTIP="192.168.37.50/32"

    # Internal network address (in CIDR notation)
    INTNET="192.168.37.0/24"

    # The address of everything (in CIDR notation)
    UNIVERSE="0.0.0.0/0"


    # Enables IP forwarding
    echo 1 > /proc/sys/net/ipv4/ip_forward


    /sbin/iptables-restore <<-EOF;

    *filter
    :INPUT DROP [0:0]
    :FORWARD DROP [0:0]
    :OUTPUT DROP [0:0]

    -P INPUT DROP
    -P FORWARD DROP


    ###################################################
    # INPUT: Incoming traffic from various interfaces #
    ###################################################

    # Loopback interface is valid
    -A INPUT -i lo -s $UNIVERSE -d $UNIVERSE -j ACCEPT

    # Local interface, local machines, going anywhere is valid
    -A INPUT -i $INTIF -s $INTNET -d $UNIVERSE -j ACCEPT

    # Remote interface, claiming to be local machines, IP spoofing, get lost
    -A INPUT -i $EXTIF -s $INTNET -d $UNIVERSE -j REJECT

    # External interface, from any source, for ICMP traffic is valid
    -A INPUT -i $EXTIF -p ICMP -s $UNIVERSE -d $EXTIP -j ACCEPT

    # Allow any related traffic coming back to the MASQ server in
    -A INPUT -i $EXTIF -s $UNIVERSE -d $EXTIP -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

    # Internal interface, all incoming traffic accepted
    -A INPUT -i $INTIF -s $INTIP -j ACCEPT

    # Catch-all rule, reject anything else
    -A INPUT -s $UNIVERSE -d $UNIVERSE -j REJECT


    ####################################################
    # OUTPUT: Outgoing traffic from various interfaces #
    ####################################################

    # Workaround bug in netfilter
    -A OUTPUT -m conntrack -p icmp --ctstate INVALID -j DROP

    # Loopback interface is valid
    -A OUTPUT -o lo -s $UNIVERSE -d $UNIVERSE -j ACCEPT

    # Local interfaces, any source going to local net is valid
    -A OUTPUT -o $INTIF -d $INTNET -j ACCEPT

    # Outgoing to local net on remote interface, stuffed routing, deny
    -A OUTPUT -o $EXTIF -s $UNIVERSE -d $INTNET -j REJECT

    # Anything else outgoing on remote interface is valid
    -A OUTPUT -o $EXTIF -s $EXTIP -d $UNIVERSE -j ACCEPT

    # Allow any related traffic leaving the the MASQ server out
    -A OUTPUT -o $INTIF -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    -A OUTPUT -o $EXTIF -s $UNIVERSE -d $EXTIP -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

    # Internal interface, all traffic accepted
    -A OUTPUT -o $INTIF -s $INTIP -j ACCEPT

    # Catch all rule, all other outgoing is denied and logged
    -A OUTPUT -s $UNIVERSE -d $UNIVERSE -j REJECT


    #################################################################
    # FORWARD: Traffic that is forwarded between various interfaces #
    #################################################################

    # Accept solicited tcp packets
    -A FORWARD -i $EXTIF -o $INTIF -m conntrack --ctstate ESTABLISHED,RELATED  -j ACCEPT

    # Allow packets across the internal interface
    -A FORWARD -i $INTIF -o $INTIF -j ACCEPT

    # Forward packets from the internal network to the Internet
    -A FORWARD -i $INTIF -o $EXTIF -j ACCEPT

    # Catch-all REJECT rule
    -A FORWARD -j REJECT

    COMMIT


    #######################################################
    # Address Translations: Masquerading traffic with nat #
    #######################################################

    *nat
    :PREROUTING ACCEPT [0:0]
    :POSTROUTING ACCEPT [0:0]
    :OUTPUT ACCEPT [0:0]

    -A POSTROUTING -o $EXTIF -j MASQUERADE

    COMMIT
    EOF

Several hours have been spent configuring this setup and the iptables syntax is very finicky. It is easy to make a mistake that renders the network unusable, so be sure that you know what you are doing before changing it.

**WARNING:** If the internal or external network configurations are changed, be sure to change the variables defining the network at the top of the file as well!


# SSH Access

If something needs to be changed on the testing gateway, the best way to access it is via SSH. Log into the gatewayadmin account with the following command:

    ssh gatewayadmin@mil-com-gateway.ad.mil.ufl.edu

If you are wearing your tinfoil hat, you can verify that the server's key fingerprints are `60:72:c3:75:d9:6d:23:67:95:44:04:2a:34:a7:e5:f8` for ED25519 and `02:b7:15:04:ba:c2:91:6b:29:9c:d3:3c:c0:98:c9:d8` for RSA. This should not be required on the LAN unless some nefarious business is going on. If you don't already know the password for this device (most of you do), then you probably should not be connecting to it.