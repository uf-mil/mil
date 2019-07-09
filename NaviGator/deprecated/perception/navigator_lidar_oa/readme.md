#NaviGator Lidar Node - Ira Hill

##Quick Overview

**Required Interfaces**

	- /velodyne_points
	- /odom
	- /get_bounds

**Provided Interfaces**

	- /ogrid
	- /info_markers
	- /database/objects
	- /database/requests
	- /unclassified_markers

**Interfaces for Debugging**

	- /ira_persist
	- /ira_frame
	- /ira_pclcloud
  

##Quick Start Guide

**Starting the node**

	- rosrun navigator_lidar_oa lidar

**Resetting the node while running**

	- rosservice call /database/requests "cmd: 'reset'"

**Requesting real objects from the database**

The service is name /database/requests with the input parameter name and will return the PerceptionObject message. Here are some example calls:


	- rosservice call /database/requests "name: 'all'" 
	- rosservice call /database/requests "name: 'shooter'"
	- rosservice call /database/requests "name: 'totem'"
	- rosservice call /database/requests "name: 'scan_the_code'"
	- rosservice call /database/requests "name: 'start_gate'" 
	- rosservice call /database/requests "name: 'buoy'"
	- rosservice call /database/requests "name: 'dock'"

**Requesting fake objects from the database**


	- rosservice call /database/requests "name: 'All'" 
	- rosservice call /database/requests "name: 'Shooter'"
	- rosservice call /database/requests "name: 'AcousticPinger'"	
	- rosservice call /database/requests "name: 'Gate_1'"
	- rosservice call /database/requests "name: 'Gate_2'"
	- rosservice call /database/requests "name: 'Gate_3'"
	- rosservice call /database/requests "name: 'Scan_The_Code'"
	- rosservice call /database/requests "name: 'CoralSurvey'" 
	- rosservice call /database/requests "name: 'BuoyField'"
	- rosservice call /database/requests "name: 'Dock'"

**Setting ROI objects from the database**

ROI (region of interest) allows us to set fake markers in the database as estimates about where structures are located in the ENU world frame. This can either be done through rosservice with the cmd input or manually in RVIZ. The fake markers include: **BuoyField**, **CoralSurvey**, **FindBreak**, **AcousticPinger**, **Shooter**, **Scan\_The_Code**, **Gate\_1**, **Gate\_2**, **Gate\_3**, and **Dock**. (These can be changed/standardized later...). The syntax requires the name of the fake marker and the **x,y location in the ENU** frame after the = sign. Example calls:


	- rosservice call /database/requests "cmd: 'Shooter=-27,-69'"
	- rosservice call /database/requests "cmd: 'CoralSurvey=50.5,71.024'"

**Interactive Markers in RVIZ**

Subscribing to **/unclassified_markers** will allow RVIZ to setup interactive markers for objects found by the lidar. Right clicking on a marker will bring up the following menu:

	- Type
	- Lock
	- Color

Type allows you to change the object classification, lock will prevent the node from classifying this particular object in code, and color sets the rgb value of the object.
Note that an unknown color defaults to 0,0,0 for rgb which is black, but we do have black buoys. So a black buoy is really 0.01,0.01,0.01 in rgb. (This can be addressed later!)

The ROI fake markers will also show up in RVIZ at the origin. Simply drag these to the necessary locations.

##Key parameters for the code
These are the knobs to turn to control the behavior of the ogrid  (these will eventually become rosparams for easy editing). Most units are marked in the variable name but all variables use meters and degrees.

	const double MAP_SIZE_METERS = 1500;
	const double ROI_SIZE_METERS = 201;
	const double VOXEL_SIZE_METERS = 0.30;
	const int MIN_HITS_FOR_OCCUPANCY = 25; 
	const int MAX_HITS_IN_CELL = 125; 
	const double MAXIMUM_Z_BELOW_LIDAR = 2; 
	const double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
	const double MAX_ROLL_PITCH_ANGLE_DEG = 5.3;
	const double LIDAR_VIEW_ANGLE_DEG = 160;
	const double LIDAR_VIEW_DISTANCE_METERS = 80;
	const double LIDAR_MIN_VIEW_DISTANCE_METERS = 5.5;
	const int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
	const double MIN_OBJECT_HEIGHT_METERS = 0.075;
