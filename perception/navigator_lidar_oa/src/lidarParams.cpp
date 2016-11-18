#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500;
const double ROI_SIZE_METERS = 201;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 20; 
const int MAX_HITS_IN_CELL = 125; 
const double MAXIMUM_Z_BELOW_LIDAR = 2; 
const double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
const double MAX_ROLL_PITCH_ANGLE_DEG = 5.3;
const double LIDAR_VIEW_ANGLE_DEG = 160;
const double LIDAR_VIEW_DISTANCE_METERS = 60;
const double LIDAR_CONFIDENCE_DISTANCE_METERS = 40;
const double LIDAR_MIN_VIEW_DISTANCE_METERS = 5.5;
const int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
const double MIN_OBJECT_HEIGHT_METERS = 0.025;
const double MIN_OBJECT_SEPERATION_DISTANCE = 1.5;
const std::vector<std::string> ROIS = {"BuoyField","CoralSurvey","FindBreak","AcousticPinger","Shooter","Scan_The_Code","Gate_1","Gate_2","Gate_3","Dock"};