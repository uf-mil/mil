# Sub8 Trajectory Generator #

## Author ##
Patrick Emami - pemami@ufl.edu

## Usage ##
To launch the tgen node: 
	`roslaunch sub8_trajectory_generator trajectory_generator.launch`

To run unit tests: 
	`catkin_make run_tests_sub8_trajectory_generator`

The TGEN provides a service, `sub8_trajectory_generator/path_plan`, that takes in a planning request. The request is two `geometry_msgs::Pose` messages, one for a start state and one for a goal state. The response is a `sub8_msgs::Path`, which is a list of `sub8_msgs::PathPoint` messages. These are comprised of a `geometry_msgs::Point` and a `float64` for the heading at each point along the path. 

Since the TGEN only does geometric planning, the resultant path needs to be profiled in order to assemble the final trajectory, which will consist of a list of waypoints (having both a `geometry_msgs::Pose` and `geometry_msgs::Twist`). 

## Alarms ##

If trajectory generation fails, the sub need not go into a meltdown. However, the rest of the system can assume that if the TGEN fails to generate a trajectory, it will raise an alarm. In response, the system can: 

* Use a straightline path and hope for the best, or
* Shutdown (float to surface)

## Miscellaneous ##

* filenames beginning with "sub8_" are extensions to the OMPL framework

