> "Every line of untested code is a line of broken code" - Kevin

Simulation is a crucial part of our development process, as it allows us to test code outside of our lake testing days. NaviGator has two simulators which are useful for testing different things:

# Gazebo
Gazebo is a full 3D simulator which can simulate all of NaviGator's sensors, including the cameras and LIDAR, in a full competition course. Currently, NaviGator's kinemetics are not simulated in gazebo so NaviGator perfectly tracts the current commanded trajectory, ignoring the controller and thruster mapper. Therefore it is quite computationally expensive and will make your computer run very slow or perhaps freeze up all together.  

> **Use Gazebo for testing full missions or perception code**.

## Usage
To run the gazebo simulator, run the launch file:

```roslaunch navigator_launch simulation.launch```

This will run both the simulator and all the other nodes normally launched on NaviGator, such as the mission server, alarms, path planner, etc. 
* By default this will open the Gazebo gui which shows this 3D environment. The gui can be disabled to save some resources by adding ```gui:=false``` to the end of the command.
* If your testing does not require the competition props, you can also run with just NaviGator in the water by adding ```sandbox:=false```

# 2D Simulator
We also have a more lightweight simulator which only simulates NaviGator's kinematics (no cameras, LiDAR, or competition props). It consumes much less RAM/CPU than Gazebo and provides a more accurate simulation of NaviGator's movements. 
> **Use the 2D simulator to test changes which do not require perception**.

## Usage
You can run the 2D simulator along with all other core nodes usually run on NaviGator by running the launch file:

```roslaunch navigator_launch sim2d.launch```

# Next Steps
Now that you have a simulator running, you can start playing around with things. A good first task is [getting NaviGator to move](Executing-simple-moves). This will verify that the simulator is working and should spark some curiosity about how this all works! 

You can also see what's going on graphically by using one of the [[Visualization]] tools.