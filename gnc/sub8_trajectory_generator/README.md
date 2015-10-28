# Sub8 Trajectory Generator

## Author
Patrick Emami - pemami@ufl.edu

To launch the tgen node: 
	`roslaunch sub8_trajectory_generator trajectory_generator.launch`

To run unit tests: 
	`catkin_make run_tests_sub8_trajectory_generator`

# TODO

* [ ] Path validity checking (batch state checking)
* [x] ODE solver (OMPL)
* [ ] RRTstar planner for controls (OMPL only has RRTstar for geometric MP)
* [ ] distance metric
* [ ] ROS alarms
* [ ] OMPL.app or simul8 for viewing paths
* [ ] Safety-path planner

# Notes

* filenames beginning with "sub8_" are extensions to the OMPL framework

