## Run The Simulator
In a new terminal, run the dynamics simulator with the command:

```roslaunch sub8_launch gazebo.launch```

## Open RVIZ
Open up RVIZ configured to display SubjuGator's topics. In a new terminal, run:

```subviz```

## Clear the initial kill
By default, the Sub will be dead in the water as a safety feature. You must manually revive the Sub. In a new terminal, run:

```aclear kill```

## Give a new waypoint
Play around with some discrete waypoints to make sure the simulation is working:

```submove down 4```: Move Sub down 4 meters

```submove f 10ft```: Move forward 10 feet

```submove yl 30deg```: Yaw left 30 degrees
