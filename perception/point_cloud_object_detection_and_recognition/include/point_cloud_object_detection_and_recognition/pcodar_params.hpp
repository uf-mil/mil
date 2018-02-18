#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace pcodar
{

extern struct pcodar_params
{
    // --- Main Params ---

    // How fast the main loop should run
    int executive_rate = 10;

    // --- Object Detection params ---
    //  -> Point cloud filtering parameters
    //  -> Point cloud clustering parameters

    // --- Point cloud filtering parameters ---

    // The number of point clouds to accrue when constructing a main mega point cloud
    int number_persistant_point_clouds = 10;

    // How much to downsample the point cloud
    float filter_points_leaf_size_x = 0.3f;
    float filter_points_leaf_size_y = 0.3f;
    float filter_points_leaf_size_z = 0.3f;

    float outier_removal_std_dev_thresh = 2.0f;
    float outier_removal_mean_k = 10;

    // The max number of points in the point cloud, this is employed when the system needs to be real time. 
    int max_number_points = 5000;


    // --- Point cloud clustering parameters ---
    
    // Minimum number of meters between two clusters for them to be seperated. (e.g. if two clusters are > than this
    // distance, it will become two objects. If they are < this distance they will become mergered into one object)
    double cluster_tolerance_m = 4.4;

    // Max and Min number of points for cluster
    int cluster_min_num_points = 2;
    int cluster_max_num_points = 25000;


    // --- Object tracking params ---

    // The max distance that two objects can be apart for them to be assigned to the same object
    double max_distance_for_association = 2.0;

    // --- O-grid params ---

    // How much to inflate the ogrid around each object
    int ogrid_inflation_cell = 2;

    // Yes these params are not technically mil_common general, however, they can be changed. 
    std::vector<std::string> object_types = {"scan_the_code", "shooter"};
    std::vector<std::string> object_colors = {"blue", "red", "green"};
} params;

void set_params(ros::NodeHandle& nh);
}
