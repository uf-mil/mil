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

### C++ 

```cpp 
ros::ServiceClient path_plan_srv = nh.serviceClient<sub8_msgs::PathPlan>("sub8_trajectory_generator/path_plan");
sub8_msgs::PathPlan pp;
geometry_msgs::Pose start_state; // initialize your starting state
geometry_msgs::Pose goal_state; // initialize your goal state
pp.request.start_state = start_state;
pp.request.goal_state = goal_state;
path_plan_srv.call(pp); // call the PathPlan service
```

* `pp.response.success` is `true` if planning succeeded
* `pp.response.path` is a `sub8_msgs::Path` msg, which is a list of `sub8_msgs::PathPoint`'s. Hence, a `Path` msg consists of a collection of `geometry_msgs::Point` and `float64` msgs, which contain the position and heading of the sub at a specific point along the path. 

## Alarms ##

The TGEN raises an alarm called `sub8_trajectory_generator_planning_failed` if something goes wrong such that the TGEN is unable to produce a trajectory from the start to goal point. 

## Miscellaneous ##

* filenames beginning with "sub8_" are extensions to the OMPL framework

