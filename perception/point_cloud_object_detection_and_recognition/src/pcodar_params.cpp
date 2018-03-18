#include <point_cloud_object_detection_and_recognition/pcodar_params.hpp>

namespace pcodar
{
pcodar_params params;
// std::vector<Eigen::Vector2d> boundary = {Eigen::Vector2d(-10, -10), Eigen::Vector2d(-10, 10), Eigen::Vector2d(100, -100), Eigen::Vector2d(100, 100)};
std::vector<Eigen::Vector2d> boundary = {Eigen::Vector2d(-210, -175), Eigen::Vector2d(-210, 15), Eigen::Vector2d(-50, 15), Eigen::Vector2d(-50, -175)};
void set_params(ros::NodeHandle& nh)
{
    nh.getParam("/pcodar/executive_rate", params.executive_rate);
    nh.getParam("/pcodar/ogrid_inflation_cell", params.ogrid_inflation_cell);
    nh.getParam("/pcodar/object_types", params.object_types);
    nh.getParam("/pcodar/object_colors", params.object_colors);
    nh.getParam("/pcodar/number_persistant_point_clouds", params.number_persistant_point_clouds);
    nh.getParam("/pcodar/filter_points_leaf_size_x", params.filter_points_leaf_size_x); 
    nh.getParam("/pcodar/filter_points_leaf_size_y", params.filter_points_leaf_size_y);  
    nh.getParam("/pcodar/filter_points_leaf_size_z", params.filter_points_leaf_size_z); 
    
    nh.getParam("/pcodar/outier_removal_std_dev_thresh", params.outier_removal_std_dev_thresh);  
    nh.getParam("/pcodar/outier_removal_mean_k", params.outier_removal_mean_k);  


    nh.getParam("/pcodar/cluster_tolerance_m", params.cluster_tolerance_m); 
    nh.getParam("/pcodar/cluster_min_points", params.cluster_min_num_points);
    nh.getParam("/pcodar/cluster_max_points", params.cluster_max_num_points); 
    nh.getParam("/pcodar/max_number_points", params.max_number_points); 
    nh.getParam("/pcodar/max_distance_for_association", params.max_distance_for_association); 
}

} // namespace pcodar
