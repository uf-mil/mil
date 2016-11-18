#ifndef LIDARPARAMS_H
#define LIDARPARAMS_H

#include <vector>
#include <string>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern const double MAP_SIZE_METERS;
extern const double ROI_SIZE_METERS;
extern const double VOXEL_SIZE_METERS;
extern const int MIN_HITS_FOR_OCCUPANCY;
extern const int MAX_HITS_IN_CELL;
extern const double MAXIMUM_Z_BELOW_LIDAR;
extern const double MAXIMUM_Z_ABOVE_LIDAR;
extern const double MAX_ROLL_PITCH_ANGLE_DEG;
extern const double LIDAR_VIEW_ANGLE_DEG;
extern const double LIDAR_VIEW_DISTANCE_METERS;
extern const double LIDAR_CONFIDENCE_DISTANCE_METERS;
extern const double LIDAR_MIN_VIEW_DISTANCE_METERS;
extern const int MIN_LIDAR_POINTS_FOR_OCCUPANCY;
extern const double MIN_OBJECT_HEIGHT_METERS;
extern const double MIN_OBJECT_SEPERATION_DISTANCE;
extern const std::vector<std::string> ROIS;
extern const double MIN_GATE_SEPERATION;
extern const double MAX_GATE_SEPERATION;
extern const double MAX_GATE_ERROR_METRIC;
extern const int MIN_HITS_FOR_VOLUME;
extern const int OBJECT_INFLATION_PARAMETER;

#endif