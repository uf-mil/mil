Operator's Guide
================

This is a guide for operating the sub.

# Startup on the Sub

```shell
roslaunch sub8_launch sub8.launch
```

At time of writing, cameras and perception are not yet stable enough to live inside the main launch. They someday will.

```shell
roslaunch sub8_launch cameras.launch
roslaunch sub8_launch perception.launch
```


# Commading a wrench manually

```shell
rostopic pub /wrench geometry_msgs/WrenchStamped (tab-tab)
```

# Commanding a thruster manually

This lets you publish a list of thrusts. For just one, supply the thruster name and a force (in Newtons) that you'd like to apply.

```shell
rostopic pub /thusters/thrust /sub8_msgs/Thrust (tab-tab)
```

# Removing the startup kill

We are currently leaning on the very ugly legacy kill system. To remove the startup kill, do

```shell
rosrun kill_handling clear
```

Unkilling is broken in some cases. We have not exerted effort to fix this, because the kill system is to be deprecated.
