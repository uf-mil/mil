
#**RobotX Position Controller Node**

**This file is used to take the trajectory given by the path planner
and command the wrenches to be given to the thruster mapper that will keep
the boat on it's desired position**

####**Glossary**:
* **wrench -** A force and a torque. Reference the info on screw theory in the [[Helpful Reading]] to see about      what a wrench is. For the purposes of NaviGator it is the pair of linear and angular vectors that describe the net movement of the vehicle. 
* **body frame -** The coordinate system with the center of the boar placed at (0,0)
* **world frame -** The coordinate system with the location of the boat at startup as (0,0)

####**Method Overview**:
* Get desired position -> ROS message callback
* Get current position -> ROS message callback
* Computer linear and angular error in world frame
* Convert the errors to the body frame using defined transformation matrix
* Use PID controller to compute desired wrench based on the body frame errors and send to the thruster mapper

####**Details and Justification:**
TODO
   