#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double MAP_SIZE_METERS = 1500;
double ROI_SIZE_METERS = 201;
double VOXEL_SIZE_METERS = 0.30;
int MIN_HITS_FOR_OCCUPANCY = 20; 
int MAX_HITS_IN_CELL = 125; 
double MAXIMUM_Z_BELOW_LIDAR = 2; 
double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
double MAX_ROLL_PITCH_ANGLE_DEG = 2.5;
double LIDAR_VIEW_ANGLE_DEG = 160;
double LIDAR_VIEW_DISTANCE_METERS = 60;
double LIDAR_CONFIDENCE_DISTANCE_METERS = 40;
double LIDAR_MIN_VIEW_DISTANCE_METERS = 6.0;
int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
double MIN_OBJECT_HEIGHT_METERS = 0.075;
double MIN_OBJECT_SEPERATION_DISTANCE = 3.0;
std::vector<std::string> ROIS = {"BuoyField","CoralSurvey","FindBreak","AcousticPinger","Shooter","Scan_The_Code","Gate_1","Gate_2","Gate_3","Dock", "EmptySpace"};
double MIN_GATE_SEPERATION = 30;
double MAX_GATE_SEPERATION = 50;
double MAX_GATE_ERROR_METRIC = 15;
int MIN_HITS_FOR_VOLUME = 31;
int OBJECT_INFLATION_PARAMETER = 2;
