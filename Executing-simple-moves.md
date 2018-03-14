One of the first things you should learn to do on Navigator is execute simple moves.

# Setup
In order for NaviGator to execute a move, a couple things are needed
* The full gnc stack must be running. This is included in the master launch or in either of the simulator launches.
* The mission server must be running, as move commands are a type of mission. This is included in the master launch or in either of the simulator launches.
* NaviGator must [not be killed](Kill-and-Alarms). NaviGator starts killed and can be revived with ```aclear kill```

# Using the move command
We have an alias, ```nmove```, which will execute the move mission with 