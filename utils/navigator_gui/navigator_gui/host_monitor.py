#!/usr/bin/env python

'''
Host Monitor: A simple node to monitor and publish the IP address status of the
core hosts on the platform's network.
'''


import socket
import subprocess

from navigator_msgs.msg import Hosts
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("host_monitor")


class HostMonitor():

    def __init__(self):
        self.pub_hosts = rospy.Publisher("/host_monitor", Hosts, queue_size=1)

        # Build an ordered list of host dictionaries that resolve to devices on navigator
        host_list = [
            "mil-nav-ubnt-wamv",
            "mil-nav-ubnt-shore",
            "mil-nav-wamv",
            "mil-com-velodyne-vlp16",
            "mil-com-sick-lms111"
        ]
        host_template = {
            "hostname": "",
            "ip": "Unknown",
            "status": "Unknown"
        }
        self.hosts = []
        for host in host_list:
            host_entry = host_template.copy()
            host_entry["hostname"] = host
            self.hosts.append(host_entry)

    def check_hosts(self):
        '''
        Resolves the hostnames of the devices on NaviGator to IP addresses on a
        10s timer. If resolution is successful, pings the IP addresses to
        check whether or not they are online.
        '''
        for host in self.hosts:

            # Resolves the IP address of the hostname
            try:
                host["ip"] = socket.gethostbyname(host["hostname"])

                # If the host is pingable, mark it as online
                try:
                    subprocess.check_output(["ping", "-c1", host["ip"]])
                    host["status"] = "Online"

                # If pinging the host is unsuccessful, mark it as offline
                except:
                    host["status"] = "Offline"

            # If hostname resolution fails, the IP address is set the unknown
            except:
                host["ip"] = "Unknown"
                host["status"] = "Unknown"

    def publish(self, event):
        '''
        Formats and publishes the list of hosts and the information gathered
        about them.
        '''
        self.check_hosts()

        # Formats the list of host dictionaries into a list of host strings
        formated_hosts = []
        for host in self.hosts:
            formated_hosts.append("{} {} {}".format(host["hostname"], host["ip"], host["status"]))

        self.pub_hosts.publish(formated_hosts)


if __name__ == "__main__":
    monitor = HostMonitor()
    rospy.Timer(rospy.Duration(10), monitor.publish, oneshot=False)
    rospy.spin()
