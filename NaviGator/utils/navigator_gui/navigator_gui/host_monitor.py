#!/usr/bin/env python

'''
Host Monitor: A simple node to monitor and publish the IP address status of the
core hosts on the platform's network.
'''


import socket
import subprocess

from navigator_msgs.msg import Host, Hosts
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("host_monitor")


class HostMonitor():

    def __init__(self):
        self.pub_hosts = rospy.Publisher("/host_monitor", Hosts, queue_size=1, latch=True)
        self.hosts = Hosts()

    def check_hosts(self):
        '''
        Resolves the hostnames of the devices on NaviGator to IP addresses on a
        10s timer. If resolution is successful, pings the IP addresses to
        check whether or not they are online.
        '''
        self.hosts = Hosts()
        for hostname in self.hosts.hostnames.split():
            host = Host()
            host.hostname = hostname

            # Resolves the IP address of the hostname
            try:
                host.ip = socket.gethostbyname(host.hostname)

                # If the host is pingable, mark it as online
                try:
                    subprocess.check_output(["ping", "-c1", host.ip])
                    host.status = "Online"

                # If pinging the host is unsuccessful, mark it as offline
                except BaseException:
                    host.status = "Offline"

            # If hostname resolution fails, the IP address is set the unknown
            except BaseException:
                host.ip = "Unknown"
                host.status = "Unknown"

            self.hosts.hosts.append(host)

    def publish(self, event):
        '''
        Publishes the list of hosts and the information gathered about them.
        '''
        self.check_hosts()
        self.pub_hosts.publish(self.hosts)


if __name__ == "__main__":
    monitor = HostMonitor()
    rospy.Timer(rospy.Duration(10), monitor.publish, oneshot=False)
    rospy.spin()
