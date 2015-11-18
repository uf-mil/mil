# Requirements #

The Trajectory Generator should...

1. Know the start state of a desired trajectory
2. Know the goal state for a trajectory
3. Know the vehicle-specific dynamics
4. Have access to the most-recently available traversability map
5. Know the location of all obstacles in its environment
6. Be able to determine the planner being used at run-time
7. Contain a module that encapsulates the construction and storage of "safety" paths in free space 
8. Be able to determine whether an updated traversability map invalidates the current trajectory
9. Make available a list of waypoints that make up the solution to the planning problem
10. Be able to handle planning and re-planning failures by switching to a safety path or requesting a system shutdown

# Theory #

### Kinematics and Dynamics ###
* [Sub dynamics equations](http://ssl.mit.edu/spheres/library/ASS2011_11-033_spheres.pdf)

### Motion-Planning ###
* [RRT* Reference](http://ijr.sagepub.com/content/30/7/846.full.pdf)
* [LQR-RRT*](http://lis.csail.mit.edu/pubs/perez-icra12.pdf)
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

## ROS msgs 
* A sub8_trajectory_msg will consist of: 
    * header 
    * list of sub8_waypoint_msgs

* A sub8_waypoint msg will consist of:
    * header 
    * geometry_msgs/Pose for position
    * geometry_msgs/Twist for velocity 

## Data input  ##
* Start state
* Goal state

## Data output
* List of waypoints

## Alarms

> _Alarm description (severity level): response_

1. Entered unavoidable collision zone (0): abort mission
1. Planning and re-planning both failed (0): abort mission

## Main Components ##
1. **Sub8 Motion-Planning**
    1. Implementation of OMPL for AUVs
    1. Main task is to use one of OMPL's path-planning algorithms (RRT) to generate a trajectory
    1. RRT* with a distance function that has a distance metric defined in the control space is being considered. It would need to be implemented from scratch. 
1. **Re-Planner**
    1. An algorithm that uses as a metric the ratio of obstructed to unobstructed waypoints will determine how much of the current trajectory needs to be re-planned. This way, time is not wasted re-planning the entire trajectory for a small obstacle, but if a large obstacle is obstructing the majority of the trajectory, a new trajectory will be generated
    1. Stretch goal is to do partial-trajectory re-planning. For starters, just re-plan starting from the affected waypoint
    1. If re-planning fails, a safety path should be used (if safety paths are being generated). Otherwise, alert the system that planning failed
1. **TGEN Node**
    1. ROS Node
    1. Handles message-passing and TGEN initializations
1. **TGEN Manager**
    1. Facade for the OMPL-implementation

(If deemed necessary and time permits)
1. **Safety-Path Generation**
    1. Safety paths are trajectories that will keep the vehicle in free space 
    1. SPC is responsible for maintaining and updating a sorted-list of best-to-worst back-up paths that can be taken if the current trajectory becomes infeasible unexpectedly
    1. After the target trajectory is generated and the motion-planner is no longer engaged with carrying out that task, instead of having it sit idle, safety-paths would be generated
    1. It is advisable to maintain at least (2) viable "safety" paths, if possible.
    1. Note that this feature is utilized best when the vehicle is moving at speeds that make stopping in a relatively short period of time impossible

## Sequence Diagrams 

![Basic Usage](http://imgur.com/U5gxEO3.png)
![New obstacle in Traversability Map](http://imgur.com/chpn1cL.png)
[Link to enlarge](http://imgur.com/chpn1cL.png)

## Languages and Libraries ##
* C++
* [CUDA in C/C++](https://developer.nvidia.com/how-to-cuda-c-cpp)
* [OMPL](http://ompl.kavrakilab.org/)
* [OMPL - RRT*](ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTstar.html)
* [OMPL - Parallel RRT](http://ompl.kavrakilab.org/classompl_1_1geometric_1_1pRRT.html)
* [Boost](http://stackoverflow.com/questions/8851670/relevant-boost-features-vs-c11) - smart pointers, foreach, etc 
* [Eigen Quick-reference](http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt)
* [c3_trajectory_generator OLD](https://github.com/uf-mil/software-common/tree/master/c3_trajectory_generator)