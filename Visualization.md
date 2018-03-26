# Overview
By default, when running navigator's code or simulation, no graphical interfaces appear. Everything is still working under the hood, but you just can't see it! It is often helpful to see graphical representations of NaviGator's movements, cameras, LIDAR, etc. Plus, these tools look really cool.

## RVIZ
The primary visualization tool used on NaviGator is RVIZ (Ros Visualizer). This GUI allows the user to subscribe to topics like cameras, odometry, and trajectories and displays them in a 3D space. It can be annoying to remember what topics to add, so we have an alias for running RVIZ with some useful topics aready subscribed.

In a terminal, run: ```nviz```

Assuming you have a [simulation](How-To-Run-The-Simulator) running or are actually connected to NaviGator, you should see a window with some stuff like this:

![nviz](https://i.imgur.com/MHpBB22.png)

By default, the cameras are not visualized to save bandwidth. You can enable them one by one by clicking on the corresponding box on the left sidebar.

## Amonitor
To see the live-updated status of a particular alarm in a terminal, use the ros_alarms tool ```amonitor <alarm name>```. You will often want to see if NaviGator is killed ```amonitor kill```

## RQT
RQT is another ROS GUI which is plugin based. You can add different plugins to see different things. Here are some you may wish yo use:
* Robot Tools/Diagnostics Viewer: See diagnostics information for hardware and certain nodes
* MIL / Dashboard: General status GUI for NaviGator, shows things like battery voltage, wrench mode, and kill status
* Task Runner Dashboard: Run missions and see their feedback/result
