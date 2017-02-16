Setting up AWS for gazebo
=========================


* You want gazebo5
* Sometimes, you have to add the google nameserver (8.8.8.8) to /etc/resolv.conf
* You might have to manually set up /etc/apt/sources.list
* Make sure opengl is installed
* [gzweb](http://gazebosim.org/gzweb), the web Gazebo UI, is your best friend
* You'll probably have to do [this stuff](http://stackoverflow.com/questions/19856192/run-opengl-on-aws-gpu-instances-with-centos)
* (And nvidia stuff)[https://devtalk.nvidia.com/default/topic/547588/error-installing-nvidia-drivers-on-x86_64-amazon-ec2-gpu-cluster-t20-gpu-/]
* Watch out for the jeepie-jeekies!


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
# alias setrosip='export ROS_IP=`hostname -I | cut -f1 -d" "`'
alias setrosip='export ROS_IP=`wget http://ipinfo.io/ip -qO -`'

chros() {
    setrosip;
    export ROS_MASTER_URI="http://$1:11311";
}
```


# Commands
```shell
sudo apt-get install linux-image-extra-virtual
```

Might have to uninstall linux-image-extra-virtual or -...generic

Install
```
sudo apt-get install linux-headers-$(uname -r)
```

Set the busid
```
lspci | grep -i nvidia
# You'll get some numbers that look like 0:2.1, replace the . with w :
sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024 --busid=X:X:X

sudo /usr/bin/X :0 &
export DISPLAY=:0

```

# Errors

modprobe: ERROR: could not insert 'nvidia_352': Operation not permitted
do sudo apt-get install linux-headers-$(uname -r)

modprobe other error
Might have to uninstall linux-image-extra-virtual or -...generic