#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <sub8_pcl/pcl_tools.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

namespace sub {
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

float feature_radius = 0.02;
float normal_radius = 0.05;

void compute_features(PointCloudNT::Ptr& cloud, FeatureCloudT::Ptr& feature_cloud) {
  // Estimate features
  pcl::console::print_highlight("Estimating features...\n");
  FeatureEstimationT feature_estimator;
  feature_estimator.setRadiusSearch(feature_radius);
  feature_estimator.setInputCloud(cloud);
  feature_estimator.setInputNormals(cloud);
  feature_estimator.compute(*feature_cloud);
}
}
