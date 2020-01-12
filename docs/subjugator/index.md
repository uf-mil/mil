# SubjuGator
This page provides general info about developing for [RoboSub](https://robonation.org/programs/robosub/), an anual competition for autonomous submarines.
MIL has participated in RoboSub for 20+ years. Most code for this is hosted under SubjuGator/.

*NOTE: Please go through the [development guide](/docs/development/development_guide) before going through this tutorial. We will assume you are currently in the development container running a tmux session.*

## Running SubjuGator in Simulation

### Launch Gazebo
In one panel, `roslaunch sub8_launch gazebo.launch gui:=true` This will launch the gazebo simulator with a visualization of truth in the simulator. This is not a represenation of what the robot knows.

### Run RVIZ
You can visualize things by running `subviz` in a new panel. `subviz` is an [alias](https://alvinalexander.com/blog/post/linux-unix/create-aliases) which launches [RVIZ](http://wiki.ros.org/rviz) with a configuration for SubjuGator. Generally speaking, this will show what information is avalible to the robot. 

### Clear the Kill
Whenever SubjuGator is initialized, it is a state called "kill" for saftey. While in the kill state, the sub will not actuate in any way so it is safe for people the physically handle it. The sub must be put into a state of "unkill" to actually do anything. To unkill the sub, go to a new panel and run, `amonitor kill` and then hold `Shift+c` until the terminal turns green. To initiate a software kill, press the space bar into the same terminal where you unkilled.

*NOTE: Hardware sytems can also raise a kill.*

### Give a move command
Give SubjuGator a move command with `submove forward 5m` also in a new panel.

*NOTE: You can also just type `submove f 5`*

### See the current odometry
Try streaming the content of a rostopic in a new pannel by running `rostopic echo /odom`
