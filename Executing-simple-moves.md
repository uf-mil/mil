One of the first things you should learn to do on Navigator is execute simple moves. This will set a new waypoint, which is where NaviGator "wants" to be, and execute all the software needed to get NaviGator from it's current waypoint to the new one, from finding a safe path to get there to executing individual motor commands.

# Setup
In order for NaviGator to execute a move, a couple things are needed
* A source of odometry (position, orientation, and velocity) is needed. This comes from the [sensors](Navigation-and-GPS) when run on NaviGator or a [simulator](How-To-Run-The-Simulator)
* The full gnc stack must be running. This is included in the master launch or in either of the simulator launches.
* The mission server must be running, as move commands are a type of mission. This is included in the master launch or in either of the simulator launches.
* NaviGator must [not be killed](Kill-and-Alarms). NaviGator starts killed and can be revived with ```aclear kill```

# Using the move command
We have an alias, ```nmove```, which will execute the move mission with certain arguments specifying what kind of move and a distance / angle.

The general form for this command is:
```nmove <direction> <distance/angle><unit (optional)>  ...<arbitrary number of additional moves>```

Some examples:
* ```nmove f 10ft```: move forward 10 feet
* ```nmove yaw_right 1.57```: rotate in place 1.57 radians clockwise (90deg)
* ```nmove left 5 yl 30deg```: strafe left 5 meters then yaw left 30 degrees
* ```nmove backward 2.5```: move backward 2.5 meters
* ```nmove circle cw```: wait for you to click a point in RVIZ, then circle around this point clockwise
* ```nmove rviz _```: wait for you to set a pose goal in RVIZ, then move to that pose
