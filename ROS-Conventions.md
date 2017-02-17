This page outlines a few simple conventions that keep the software stack clean and easy to use.

# Timers
Timers of any kind should **ALWAYS** use rospy or roscpp time. Any robot-facing code that uses time to make decisions should use time from Rospy or Roscpp. 

This is so that we can use simulated time, and run the simulation *faster* than real-time. ROS-timers will automatically handle the weird time stuff, but Python's time.time(), and other language-native timing stuff will just use the System time. This will cause severely undesired behavior.

# Packages
Packages should be prepended with *{vehicle}*_*.
Ex:

`sub8_thruster_driver`

`navigator_msg_multiplexer`

And so on.

This is intended to guarantee that there will be no name conflicts with existing packages, or other packages with similar names on other vehicles. The prefix for general purpose software should be mil_*.

# Logging
[Take a look at the ROS logging guide](http://wiki.ros.org/roscpp/Overview/Logging)
* Make sure you have good ROS logging, with appropriate log-levels for the information logged
* You should log anything that a human reviewer would want to know in the event of a problem
* Log anything that could be considered fatal, with some description as to what happened
* If your node interacts with hardware, hardware connection/loss, kill/unkill state transitions
* Be diligent in logging, but don't feel that you need to log every single possible state change

# ROS Exceptions
In order to efficiently handle crashes within ROS nodes, we should define a *{vehicle}*Exception class that extends the ROSException base class. Then, we can create different exception types that extend *{vehicle}*Exception that can catch different types of potential failures (i.e. a *{vehicle}*NoSuchObjectException class).

These exceptions should set off an appropriate system alarm, so that each node isn't bogged down with exception handling logic. 

[ROS Exception Docs](http://docs.ros.org/api/rospy/html/rospy.exceptions-module.html)