## ROS Exceptions

In order to efficiently handle crashes within ROS nodes, we should define a Sub8Exception class that extends the ROSException base class. Then, we can create different exception types that extend Sub8Exception that can catch different types of potential failures (i.e. a Sub8NoSuchObjectException class).

These exceptions should set off an appropriate system alarm, so that each node isn't bogged down with exception handling logic. 

[ROS Exception Docs](http://docs.ros.org/api/rospy/html/rospy.exceptions-module.html)