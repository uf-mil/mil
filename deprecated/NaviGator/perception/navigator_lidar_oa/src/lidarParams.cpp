#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double MAP_SIZE_METERS = 2100;
double ROI_SIZE_METERS = 201;
double VOXEL_SIZE_METERS = 0.30;
int MIN_HITS_FOR_OCCUPANCY = 20;
int MAX_HITS_IN_CELL = 200;
int LIDAR_HITS_INCREMENT = 35;
double MAXIMUM_Z_BELOW_LIDAR = 2.25;
double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
double MAX_ROLL_PITCH_ANGLE_DEG = 2.5;
double LIDAR_VIEW_ANGLE_DEG = 160;
double LIDAR_VIEW_DISTANCE_METERS = 60;
double LIDAR_CONFIDENCE_DISTANCE_METERS = 40;
double LIDAR_MIN_VIEW_DISTANCE_METERS = 5.5;
int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
double MIN_OBJECT_HEIGHT_METERS = 0.15;
double MIN_OBJECT_SEPERATION_DISTANCE = 3.0;
std::vector<std::string> ROIS = { "BuoyField", "CoralSurvey",   "FindBreak", "AcousticPinger",
                                  "Shooter",   "Scan_The_Code", "Gate_1",    "Gate_2",
                                  "Gate_3",    "Dock",          "EmptySpace" };
double MIN_GATE_SEPERATION = 30;
double MAX_GATE_SEPERATION = 50;
double MAX_GATE_ERROR_METRIC = 15;
int MIN_HITS_FOR_VOLUME = 31;
int OBJECT_INFLATION_PARAMETER = 2;
double VOXEL_SIZE_Z_METERS = 0.15;
double VOXEL_SIZE_Z_MIN_HITS = 10;
double volumes[5][8] = { { 1.0, 1.5, 8.0, 10.0, 8.0, 10.0, 2.75, 3.25 },          // dock (NOT TESTED!)
                         { 0.75, 2.25, 3.0, 5.5, 3.0, 5.5, 2.5, 5.5 },            // shooter
                         { 0.1, 0.75, 1.3, 2.25, 1.3, 2.25, 1.7, 2.5 },           // scan_the_code
                         { -0.6, 0.0, 0.8, 1.8, 0.8, 1.8, 0.8, 1.8 },             // totems
                         { -1.25, -0.8, 0.125, 1.0, 0.125, 1.0, 0.125, 0.65 } };  // buoy
void fill_volume(std::string key, ros::NodeHandle& nh, double* v)
{
  std::vector<double> temp_volume;
  if (!nh.getParam(key, temp_volume))
    return;
  if (temp_volume.size() != 8)
    throw std::runtime_error("Wrong array size in volume param");
  std::copy(temp_volume.begin(), temp_volume.end(), v);
}
void set_params(ros::NodeHandle& nh)
{
  if (!nh.getParam("MAP_SIZE_METERS", MAP_SIZE_METERS))
  {
    std::cout << "not set " << std::endl;
  }
  nh.getParam("ROI_SIZE_METERS", ROI_SIZE_METERS);
  nh.getParam("VOXEL_SIZE_METERS", VOXEL_SIZE_METERS);
  nh.getParam("MIN_HITS_FOR_OCCUPANCY", MIN_HITS_FOR_OCCUPANCY);
  nh.getParam("MAX_HITS_IN_CELL", MAX_HITS_IN_CELL);
  nh.getParam("LIDAR_HITS_INCREMENT", LIDAR_HITS_INCREMENT);
  nh.getParam("MAXIMUM_Z_BELOW_LIDAR", MAXIMUM_Z_BELOW_LIDAR);
  nh.getParam("MAXIMUM_Z_ABOVE_LIDAR", MAXIMUM_Z_ABOVE_LIDAR);
  nh.getParam("MAX_ROLL_PITCH_ANGLE_DEG", MAX_ROLL_PITCH_ANGLE_DEG);
  nh.getParam("LIDAR_VIEW_ANGLE_DEG", LIDAR_VIEW_ANGLE_DEG);
  nh.getParam("LIDAR_VIEW_DISTANCE_METERS", LIDAR_VIEW_DISTANCE_METERS);
  nh.getParam("LIDAR_CONFIDENCE_DISTANCE_METERS", LIDAR_CONFIDENCE_DISTANCE_METERS);
  nh.getParam("LIDAR_MIN_VIEW_DISTANCE_METERS", LIDAR_MIN_VIEW_DISTANCE_METERS);
  nh.getParam("MIN_LIDAR_POINTS_FOR_OCCUPANCY", MIN_LIDAR_POINTS_FOR_OCCUPANCY);
  nh.getParam("MIN_OBJECT_HEIGHT_METERS", MIN_OBJECT_HEIGHT_METERS);
  nh.getParam("MIN_OBJECT_SEPERATION_DISTANCE", MIN_OBJECT_SEPERATION_DISTANCE);
  nh.getParam("MIN_GATE_SEPERATION", MIN_GATE_SEPERATION);
  nh.getParam("MAX_GATE_SEPERATION", MAX_GATE_SEPERATION);
  nh.getParam("MAX_GATE_ERROR_METRIC", MAX_GATE_ERROR_METRIC);
  nh.getParam("MIN_HITS_FOR_VOLUME", MIN_HITS_FOR_VOLUME);
  nh.getParam("OBJECT_INFLATION_PARAMETER", OBJECT_INFLATION_PARAMETER);
  nh.getParam("VOXEL_SIZE_Z_METERS", VOXEL_SIZE_Z_METERS);
  nh.getParam("VOXEL_SIZE_Z_MIN_HITS", VOXEL_SIZE_Z_MIN_HITS);
  fill_volume("VOLUMES/DOCK", nh, volumes[0]);
  fill_volume("VOLUMES/SHOOTER", nh, volumes[1]);
  fill_volume("VOLUMES/SCAN_THE_CODE", nh, volumes[2]);
  fill_volume("VOLUMES/TOTEM", nh, volumes[3]);
  fill_volume("VOLUMES/BUOY", nh, volumes[4]);
  if (nh.hasParam("ROIS"))
  {
    ROIS.clear();
    nh.getParam("ROIS", ROIS);
  }
}
