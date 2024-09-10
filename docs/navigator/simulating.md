# Simulating NaviGator

If we had to test all of our new code on the physical build of NaviGator each time
we made a change, we'd never be able to make changes quickly! Therefore, when
making changes, it's recommended to simulate them before deploying NaviGator
in a physical testing environment. Luckily, we have a great simulation environment,
powered by VRX.

## VRX

[VRX](https://github.com/osrf/vrx) is a simulation environment for aquatic unmanned vehicles, developed by the
[Open Source Robotics Foundation](https://www.openrobotics.org/). It was originally
developed for the RobotX competition, and it has continued development in strong
collaboration with RoboNation, the organization running RobotX. Eventually, the
VRX competition was launched, which was an exclusively-virtual robotics competition
that used the VRX platform as a simulator.

VRX is a powerful simulator built on top of [Gazebo](https://gazebosim.org), a standard
simulator for ROS environments. In VRX, you can customize the vehicle used in the simulator,
as well as the surrounding environmental conditions (such as the wind, waves, fog,
and ambient light). Also, the VRX platform typically provides world files and object
models that are similar to actual task elements used in the biannual international RobotX competition,
which MIL competes in.

## Launching the NaviGator simulation

To launch the simulation for NaviGator, run the following command in your terminal:
```bash
roslaunch navigator_launch simulation.launch --screen
```

There are some variables you can provide to this launch file to customize the
simulation environment. These include:

* `use_mil_world`: If `true`, use world files from `navigator_gazebo` (our own
    ROS package built for NaviGator). Otherwise, use world files from `vrx_gazebo`
    (aka, world files provided to everyone in VRX). Default: `false`.
* `world`: Which world file to launch. Typically, there are several world files,
    each of which will focus on one part of the competition.

## Viewing and inspecting the environment

Now, you've launched the simulation environment, but you're not actually able
to see NaviGator -- what gives? At this point, you can run one of two commands.

```bash
gazebogui
```

`gazebogui` will launch Gazebo, which will provide a beautiful, high-level view
of the simulation.

```bash
nviz
```

`nviz` will launch Rviz, which will give you a much more granular view of NaviGator.
Here, you can inspect the frames and joints that make up NaviGator, the inputs of
its sensors, etc.

## Doing things

At this point, it should be just like NaviGator is in the lake -- you can run
missions, move NaviGator around using `nmove`, or take a peek at the cameras/sensors.
