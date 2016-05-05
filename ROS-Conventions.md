ROS Conventions
================

# Timers
Timers of any kind should **ALWAYS** use rospy or roscpp time. Any robot-facing code that uses time to make decisions should use time from Rospy or Roscpp. 

This is so that we can use simulated time, and run the simulation *faster* than real-time. ROS-timers will automatically handle the weird time stuff, but Python's time.time(), and other language-native timing stuff will just use the System time. This will cause severely undesired behavior.

# Packages

Packages should be prepended with sub8_*.
Ex:

`sub8_thruster_driver`

`sub8_control`

And so on.

This is intended to guarantee that there will be no name conflicts with existing packages, or other packages with similar names. I.e., to create a "sub8" ROS namespace.