# 2D Simulator for NaviGator

Launched through the command:

roslaunch owl.launch

The 2D Simulator subscribes to '/wrench'. This is the forces that the boat will be experiencing to reach a target destination. It publishes an odometry message to '/odom'. 

This can be used to test basic scripts for NaviGator and changes to the GNC. 

TODO list: 

[] simulate GPS
[] simulate absodom
[] display more meaningful data
[] refine current parameters (i.e. weight of the boat, the drag, and so on.)