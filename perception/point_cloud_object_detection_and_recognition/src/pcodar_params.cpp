#include <point_cloud_object_detection_and_recognition/pcodar_params.hpp>

namespace pcodar {
pcodar_params params;

boundary_t boundary = {
    Eigen::Vector3d(-80, -175, 0), Eigen::Vector3d(-100, -30, 0),
    Eigen::Vector3d(100, 30, 0), Eigen::Vector3d(100, -175, 0)};

void set_params(ros::NodeHandle& nh) {
  nh.getParam("executive_rate", params.executive_rate);
  nh.getParam("ogrid_inflation_cell", params.ogrid_inflation_cell);
  nh.getParam("object_types", params.object_types);
  nh.getParam("object_colors", params.object_colors);
  nh.getParam("number_persistant_point_clouds",
              params.number_persistant_point_clouds);
  nh.getParam("filter_points_leaf_size_x",
              params.filter_points_leaf_size_x);
  nh.getParam("filter_points_leaf_size_y",
              params.filter_points_leaf_size_y);
  nh.getParam("filter_points_leaf_size_z",
              params.filter_points_leaf_size_z);

  nh.getParam("remove_points_near_lidar_distance",
              params.remove_points_near_lidar_distance);

  nh.getParam("outier_removal_std_dev_thresh",
              params.outier_removal_std_dev_thresh);
  nh.getParam("outier_removal_mean_k", params.outier_removal_mean_k);

  nh.getParam("cluster_tolerance_m", params.cluster_tolerance_m);
  nh.getParam("cluster_min_points", params.cluster_min_num_points);
  nh.getParam("cluster_max_points", params.cluster_max_num_points);
  nh.getParam("max_number_points", params.max_number_points);
  nh.getParam("max_distance_for_association",
              params.max_distance_for_association);
}

}  // namespace pcodar
