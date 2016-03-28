# **Networking with NaviGator**

To operate the boat we need a way to connect to it while it is on the lake so that we can see the ROS messages flowing through the system. Yes, ROS is amazing, it allows us to subscribe to the ROS master via IP address. So through some basic networking we are able to connect N number of devices to the same master. It is exactly in this manner that we test and run the boat. The nodes that are finalized are launched on the boat and only the nodes being experimented with are launched on the shore, allowing us to start/stop/make changes/all without bringing the entire system down. 

## **There are a 3 steps to networking:**

### **1. Setting a static IP with the router**

**Before beginning make sure you are plugged into the NaviGator shore router**

Hover over the wireless symbol and click on 'edit connections'

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/one_zps0ould2e9.png)

Click on 'add'

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/two_zpsskxie77y.png)

Select 'ethernet' and create the connection

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/three_zpsnqcmg1jo.png)

Name your connection

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/four_zps9ptvczfr.png)

Select the HW address of your ethernet adapter - should only be one in the dropdown, it's that one

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/five_zpsunmrk4ef.png)

Move to the IPv4 settings and set the 'Method' section to 'manual'

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/six_zpsbl5sihv7.png)

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/seven_zpsr7vfldgn.png)

![](http://i909.photobucket.com/albums/ac298/Zach_Goins/eight_zpslnmbv5sx.png)

**IMPORTANT:** Email Zach at zach.a.goins@gmail.com with the IP address you chose so that he can add it to the hosts list on the boat as well. 

### **2. Adding the necessary hosts to your host list**

Open a new terminal and run:

    sudo nano /etc/hosts

Add the following lines to this file in the list of hosts
	
    192.168.1.101   navigator
    192.168.1.2     nav_router_onboard
    192.168.1.20    nav_router_onshore



		


* Adding the necessary ROS master in your .bashrc
