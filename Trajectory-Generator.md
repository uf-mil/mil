# Terminology #

* **Point**: 
    1. In the **body coordinate frame**, a **point** is a 6D vector describing the sway, surge, heave, roll, pitch, and yaw of the vehicle at an instance in time
    1. In the **world coordinate frame**, a **point** is a 6D vector describing the x, y, and z (depth) positions and the angular velocities about each axis of the vehicle at an instance in time
* **Waypoint**: A desired position (location and orientation) and desired velocity (linear and angular) within the world reference frame
* **Free space**: A sub-region within the visible, traversable region near the vehicle that is believed to be unobstructed 
* **RRT**: Rapidly-Exploring Random Trees - [Seminal paper on RRTs](http://webpages.uncc.edu/xiao/itcs6151-8151/RRT.pdf)

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
* [FLANN Kd-trees doc](http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann_visapp09.pdf)
* [FLANN download](http://www.cs.ubc.ca/research/flann/)
* [Speeding up RRT* with a GPU](http://sertac.scripts.mit.edu/web/wp-content/papercite-data/pdf/bialkowski.karaman.ea-iros11.pdf)

### Splines ###
* [Much Ado About Splines](http://graphics.stanford.edu/courses/cs348a-12-winter/Handouts/handout27.pdf)
* [Fast Smoothing of Motion Planning Trajectories using B-Splines](https://wwwx.cs.unc.edu/~panj/index_files/files/ICRA11.pdf)

# System Details #

## Configuration ##
* Any kinematic/dynamic constraints that can be provided ahead-of-time to the T-generator
* TBD

## Data input  ##
* Starting point 
* Variable limitations on the trajectory generation (TBD)

## Data output ##
* Next waypoint

## Modules ##
1. Motion-Planner
1. Plan-Smoother
1. Safety-Path Generator
1. Re-Planner
1. T-Generator Facade 

## Sequence Diagrams ##
