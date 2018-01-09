#include <point_cloud_object_detection_and_recognition/pcodar_params.hpp>

namespace pcodar
{

void set_params(ros::NodeHandle& nh, pcodar_params& params)
{
    nh.getParam("/pcodar/executive_rate", params.executive_rate);
    nh.getParam("/pcodar/ogrid_inflation_cell", params.ogrid_inflation_cell);
    nh.getParam("/pcodar/object_types", params.object_types);
    nh.getParam("/pcodar/object_colors", params.object_colors);
    nh.getParam("/pcodar/number_persistant_point_clouds", params.number_persistant_point_clouds);
    nh.getParam("/pcodar/filter_points_leaf_size_x", params.filter_points_leaf_size_x); 
    nh.getParam("/pcodar/filter_points_leaf_size_y", params.filter_points_leaf_size_y);  
    nh.getParam("/pcodar/filter_points_leaf_size_z", params.filter_points_leaf_size_z);  
    nh.getParam("/pcodar/cluster_tolerance_m", params.cluster_tolerance_m); 
    nh.getParam("/pcodar/cluster_min_points", params.cluster_min_num_points);
    nh.getParam("/pcodar/cluster_max_points", params.cluster_max_num_points); 
    nh.getParam("/pcodar/max_number_points", params.max_number_points); 
    nh.getParam("/pcodar/max_distance_for_association", params.max_distance_for_association); 
}

} // namespace pcodar
