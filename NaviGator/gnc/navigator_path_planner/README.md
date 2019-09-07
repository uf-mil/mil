# Navigator Path Planner
Contains the node and messages for NaviGator's path planner
and trajectory generator, which accepts pose goals
and generates feasable paths to that goal which does not collide
with objects in our occupancy grid. The node also
creates a trajectory to follow this path, used by the controller.


## Implementation
Currently, NaviGator uses a RRT based path planner known as lqRRT,
written by a student in the lab. See the included submodule
in gnc/navigator_path_planner/lqRRT for the library
used. The ros interface (lqrrt_node.py), started as a clone
from the ROS demo in lqRRT, but will continue to be developed
independently as NaviGator-specific needs are addressed.
