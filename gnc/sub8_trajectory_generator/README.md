# Sub8 Trajectory Generator

## Author
Patrick Emami - pemami@ufl.edu

To launch the node (currently does nothing): 
	`roslaunch sub8_trajectory_generator trajectory_generator.launch`

To run unit tests: 
	`catkin_make run_tests_sub8_trajectory_generator`

# TODO

* [ ] Path validity checking (batch state checking)
* [ ] ODE solver (OMPL)
* [ ] RRTstar planner for controls (OMPL only has RRTstar for geometric MP)
* [ ] LQR as an RRTstar heuristic
* [ ] ROS alarms and msgs
* [ ] OMPL.app for viewing paths
* [ ] Re-planning component 
* [ ] Safety-path planner


