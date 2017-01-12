Operator's Guide
================

This is a guide for operating the sub.

# Startup on the Sub

A self checking node exists to run a system check on everything the sub needs to preform nominally. For the most reliable results, run this check whilst in the water:
```shell
rosrun sub8_launch self_check.py
```

It will prompt you to make sure certain certain launches are running as well. In a tmux or screen session on the sub, run:
```shell
roslaunch sub8_launch sub8.launch
```
and on a terminal on a shore computer (that has its `ROS_MASTER_URI` set to the sub) run:
```shell
roslaunch sub8_launch shore_control.launch
```
The self check should complete, and hopefully almost everything passed! The next step is clearing the initial kill that is raised on launch. **NOTE: CLEARING THIS KILL WILL START SPINNING PROPS AND CAUSE THE SUB TO MOVE.** Usually it is best to check in rviz that odom looks stable before doing this and not be near any walls.
```shell
rosrun ros_alarms clear kill
# Or if you have .sub_aliases aliased:
aclear kill
```

At time of writing, perception is not yet stable enough to live inside the main launch. Someday it will.

```shell
roslaunch sub8_launch perception.launch
```

Happy Subbing!

# Commanding a wrench manually

Sometimes you will want to check to make sure all the props are moving the in right direction in the pool or on land in the lab, in these cases publishing a direct wrench to be executed is desired. Do so as follows:

```shell
rostopic pub /wrench geometry_msgs/WrenchStamped (tab-tab)
```

_Note: as per standard connection +x is forward, +y is left, and +z is up._

# Commanding a thruster manually

This lets you publish a list of thrusts. For just one, supply the thruster name and a force (in Newtons) that you'd like to apply.

```shell
rostopic pub /thusters/thrust /sub8_msgs/Thrust (tab-tab)
```
