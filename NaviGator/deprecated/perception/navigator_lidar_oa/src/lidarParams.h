#ifndef LIDARPARAMS_H
#define LIDARPARAMS_H

#include <ros/ros.h>
#include <string>
#include <vector>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern double MAP_SIZE_METERS;
extern double ROI_SIZE_METERS;
extern double VOXEL_SIZE_METERS;
extern int MIN_HITS_FOR_OCCUPANCY;
extern int MAX_HITS_IN_CELL;
extern int LIDAR_HITS_INCREMENT;
extern double MAXIMUM_Z_BELOW_LIDAR;
extern double MAXIMUM_Z_ABOVE_LIDAR;
extern double MAX_ROLL_PITCH_ANGLE_DEG;
extern double LIDAR_VIEW_ANGLE_DEG;
extern double LIDAR_VIEW_DISTANCE_METERS;
extern double LIDAR_CONFIDENCE_DISTANCE_METERS;
extern double LIDAR_MIN_VIEW_DISTANCE_METERS;
extern int MIN_LIDAR_POINTS_FOR_OCCUPANCY;
extern double MIN_OBJECT_HEIGHT_METERS;
extern double MIN_OBJECT_SEPERATION_DISTANCE;
extern std::vector<std::string> ROIS;
extern double MIN_GATE_SEPERATION;
extern double MAX_GATE_SEPERATION;
extern double MAX_GATE_ERROR_METRIC;
extern int MIN_HITS_FOR_VOLUME;
extern int OBJECT_INFLATION_PARAMETER;
extern double VOXEL_SIZE_Z_METERS;
extern double VOXEL_SIZE_Z_MIN_HITS;
extern double volumes[5][8];
extern void set_params(ros::NodeHandle& nh);
#endif
