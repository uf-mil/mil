# About sjtu_drone #

sjtu_drone is a quadrotor simulation program forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator) , which is developed with ROS + Gazebo. It is used for testing visual SLAM algorithms aiding with different sensors, such as IMU, sonar range finder and laser range finder. Here by 'sjtu', it means Shanghai Jiao Tong University. Currently, this program is used for testing algorithms for [UAV contest in SJTU](http://mediasoc.sjtu.edu.cn/wordpress)

# Requirements #
The package requires ROS indigo version (which means you'd better use Ubuntu 14.04 or Linux Mint 17 or the related later versions). The Gazebo version is suggested to be larger than 3.0. As the default version of Gazebo coming with ROS indigo is 2.0, it is suggested that do not use the full installation but the desktop installation.
```
$sudo apt-get install ros-hydro-desktop //I think this is outdated, should be sudo apt-get install ros-indigo-desktop
```
#Download and Compiling#
```
$cd <catkin_ws>/src
$git clone https://dannis@bitbucket.org/dannis/sjtu_drone.git
$cd <catkin_ws>
$catkin_make
```

Here <catkin_ws> is the path of the catkin work space. Please refer to the [tutorial](http://wiki.ros.org/ROS/Tutorials) about how to create a catkin work space in ROS.

#Run
The simplest way is calling
```
$cd <where you check out the code>
$export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
$roslaunch sjtu_drone start.launch
```
or running the different parts of the package step by step

```
$cd <where you check out the code>
$export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
$roscore #to start the ROS server
$rosrun sjtu_drone start_gzserver <world file> #run the gazebo server and loading the world file
$rosrun sjtu_drone start_gui #run the gazebo client
$rosrun sjtu_drone spawn_model # generate a quadrotor in the scene
$rosrun sjtu_drone drone_keyboard # run the keyboard controller to control the quadrotor
```

# Read sensor data from ROS topics #
```
forward looking camera :  /drone/front_camera/image_raw
downward looking camera: /drone/down_camera/image_raw
sonar data:  /drone/sonar
laser range data: /drone/laser
```