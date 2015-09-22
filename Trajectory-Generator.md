# Terminology #

* **Point**: 
    1. In the **body coordinate system**, a **point** is a 6D vector describing the sway, surge, heave, roll, pitch, and yaw of the vehicle at an instance in time
    1. In the **world coordinate system**, a **point** is a 6D vector describing the x, y, and z (depth) positions and the angular rotation about each axis (read: quaternion) of the vehicle at an instance in time
* **Waypoint**: A desired position and desired velocity within the world coordinate system
* **RRT**: [Rapidly-Exploring Random Trees] (http://webpages.uncc.edu/xiao/itcs6151-8151/RRT.pdf)
* **Workspace**: The physical space that the robot operates in. It is assumed that the boundary of the
workspace represents an obstacle for the robot
* **State space**: The parameter space for the robot. This space represents all possible configurations
of the robot in the workspace. A single point in the state space is a state
* **Free state space**: A subset of the state space in which each state corresponds to an obstacle free
configuration of the robot embedded in the workspace
* **Path**: A continuous mapping of states in the state space. A path is collision free if each element of
the path is an element of the free state space

[source](http://ompl.kavrakilab.org/OMPL_Primer.pdf) 

# Requirements #

The Trajectory Generator should...

1. Know the starting point in world coordinates for a trajectory that needs to be calculated
2. Know the target waypoint for a trajectory
3. Know the desired speed for the trajectory, whether given or pre-defined 
4. Know the vehicle-specific kinematics and dynamics to parameterize the motion-planner (MP) with 
5. Have the most-recently available traversability map and should know the location of all obstacles on the map
6. Contain a motion-planner (MP) module that has multiple, swappable planning algorithms
7. Contain a module that is responsible for smoothing the path produced by the MP (Ensuring C3 continuity) 
8. Contain a module that encapsulates the construction and storage of "safety" back-up paths in free space generated and updated by the MP
9. Generate a list of waypoints that make up the resultant trajectory; the trajectory should be C3 continuous and obey vehicle-specific kinodynamics
10. Provide nodes subscribed to the T-generator with the next viable waypoint in a "timely manner" (_Need to define the rate at which waypoints are served_)
11. Be able to receive an updated traversability map with new obstacles, and assess whether a portion of
the list of future waypoints will collide with the new obstacles
12. Be able to handle planning and re-planning failures by switching to a safety path or requesting a system shutdown when appropriate 

# Theory #

### Kinematics and Dynamics ###
* [Kinematics and Dynamics for AUVs](http://www.mate.tue.nl/mate/pdfs/10894.pdf)

### Motion-Planning ###
* [RRT* Reference](http://ijr.sagepub.com/content/30/7/846.full.pdf)
* [Why RRT* is awesome, and why the distance metric and node-extension heuristics are key](http://lis.csail.mit.edu/pubs/perez-icra12.pdf)
* [Speeding up RRT* with a GPU](http://sertac.scripts.mit.edu/web/wp-content/papercite-data/pdf/bialkowski.karaman.ea-iros11.pdf)
* [Safety Paths with RRT*-AR](https://www.ri.cmu.edu/pub_files/2013/5/RRTS_AR.pdf)
* [MIT's RRT planner from the DUC](http://acl.mit.edu/papers/KuwataTCST09.pdf)
* [LQR](http://ocw.mit.edu/courses/mechanical-engineering/2-154-maneuvering-and-control-of-surface-and-underwater-vehicles-13-49-fall-2004/lecture-notes/lec19.pdf)

### Splines ###
* [Much Ado About Splines and CX Continuity](http://graphics.stanford.edu/courses/cs348a-12-winter/Handouts/handout27.pdf)
* [Fast Smoothing of Motion Planning Trajectories using B-Splines](https://wwwx.cs.unc.edu/~panj/index_files/files/ICRA11.pdf)

# 3D Occupancy Grid
[OctoMap](http://www2.informatik.uni-freiburg.de/~hornunga/pub/hornung13auro.pdf)

# System Details #

## Configuration ##
* Any kinematic and/or dynamic constraints that can be provided ahead-of-time to the T-Generator
* TODO

## ROS msgs 
* A sub8_waypoint msg will consist of:
    * header 
    * geometry_msgs/PoseWithCovariance for position
    * geometry_msgs/Twist for velocity 

## Data input  ##
* Starting waypoint for trajectory - query for new trajectory should be received via ROS actionlib
* End waypoint for trajectory
* Variable vehicle-specific limitations on the trajectory generation

## Data output
* Next viable waypoint - Should be a ROS topic?

## Alarms

> _Alarm description (severity level): response_

1. Entered unavoidable collision zone (0): abort mission
1. No usable safety paths (1): abort mission
1. Planning failed (2): try to use a safety path
1. Re-plan failure (2): try to use a safety path 

## Components ##
1. **Motion-Planner**
    1. Implementation of OMPL for AUVs
    1. Main task is to use one of OMPL's path-planning algorithm to generate a trajectory (RRT*)
    1. LQR-RRT* is being considered, which uses LQR as a distance function and node extension heuristic in RRT* for increased performance
1. **Fast-Smoother**
    1. Takes in a path (list of waypoints) generated by the motion-planner and applies (a) smoothing algorithm(s) to create a feasible trajectory
    1. May contain more than one smoothing algorithm 
    1. Outputs the smoothed trajectory (list of waypoints)
1. **Safety-Path Coordinator**
    1. Safety paths are trajectories that will keep the vehicle in free space 
    1. SPC is responsible for maintaining and updating a library of back-up paths that can be taken if the live trajectory becomes infeasible unexpectedly
    1. After the target trajectory is generated and the motion-planner is no longer engaged with carrying out that task, instead of having it sit idle, that module should begin generating and updating safety paths 
    1. It is advisable to maintain at least (2) viable "safety" paths, if possible.
    1. SPC should provide options for returning the "best" safety-path: the path that keeps the vehicle farthest from any obstacles, the path that will get the vehicle to a safe location in the shortest time, the path that expends the least amount of energy, etc. **This probably isn't necessary for the MVP**
1. **Re-Planner**
    1. This module will utilize the functionality of the motion-planner, and will only encapsulate algorithms that control the re-planning logic 
    1. When the T-Generator is handed a new traversability map and told that there are new obstacles, the motion-planner will be queried to assess whether the new obstacles will obstruct the current trajectory at any point
    1. If the current trajectory is obstructed, the motion-planner will begin to attempt to replan
    1. An algorithm that uses as a metric the ratio of obstructed to unobstructed waypoints will determine how much of the current trajectory needs to be re-planned. This way, time is not wasted re-planning the entire trajectory for a small obstacle, but if a large obstacle is obstructing the majority of the trajectory, a new trajectory will be generated
    1. If re-planning fails, the SPC should be notified to provide a safety path 
1. **Message-Handler / ROS Node "main"**
    1. Encapsulates all topic/service/action logic and callbacks 
    1. Should be loosely coupled from all other modules - only needs to know the T-Generator Mediator, who will handle delegating tasks to the other modules
1. **T-Generator Mediator**
    1. Encapsulates all communication between T-Generator modules to promote loose-coupling  

## Languages and Libraries ##
* C++
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 
* [CUDA in C/C++](https://developer.nvidia.com/how-to-cuda-c-cpp)
* [OMPL](http://ompl.kavrakilab.org/)
* [OMPL - RRT*](ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTstar.html)
* [OMPL - Parallel RRT](http://ompl.kavrakilab.org/classompl_1_1geometric_1_1pRRT.html)
* [Boost](http://stackoverflow.com/questions/8851670/relevant-boost-features-vs-c11) - smart pointers, foreach, etc 
* [c3_trajectory_generator OLD](https://github.com/uf-mil/software-common/tree/master/c3_trajectory_generator)