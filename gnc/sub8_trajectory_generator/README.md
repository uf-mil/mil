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

# Alarms

If trajectory generation fails, the sub need not go into a meltdown. However, the rest of the system can assume that if the TGEN fails to generate a trajectory, it will raise an alarm. In response, the system can: 

* Attempt to navigate without a trajectory (?)
* Shutdown (float to surface)

Since the trajectory generator is responsible for trajectory validation, the trajectory generator will need to raise alarms if the current trajectory is invalid and the system needs to abort. If the trajectory generator can safely replan, no need to disrupt normal system ops. 