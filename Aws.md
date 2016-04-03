Setting up AWS for gazebo
=========================


* You want gazebo5, for
* Sometimes, you have to add the google nameserver (8.8.8.8) to /etc/resolv.conf
* You might have to manually set up /etc/apt/sources.list
* Make sure opengl is installed
* [gzweb](http://gazebosim.org/gzweb), the web Gazebo UI, is your best friend


# ROS IP
You can subscribe & publish to an EC2 instance

#### On the EC2 machine
* Figure out the public IP and export ROS_IP=that
    * when you start roscore, it will claim to find an error
    * export ROS_IP=`curl http://169.254.169.254/latest/meta-data/public-ipv4`


#### On your own machine

* Setet your own machine's ROS_IP
* And set your machine's rosmaster


I use...

```shell
alias setrosip='export ROS_IP=`hostname -I | cut -f1 -d" "`'
chros() {
    setrosip;
    export ROS_MASTER_URI="http://$1:11311";
}
```
