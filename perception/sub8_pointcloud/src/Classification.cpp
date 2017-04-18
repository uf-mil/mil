//TODO: Segmentation, Classification, Bounds, Ogrid filtering

#include "Classification.hpp"
Classification::Classification(ros::NodeHandle *nh)
{
  nh_ = nh;

  //TODO: Algorithms to filter pointcloud and possibly some classification scheme

  // pubTest = nh_->advertise<visualization_msgs::MarkerArray>( "/test_pub", 0 );
  // pubTest = nh_->advertise<pcl::PointCloud<pcl::PointXYZI>>("ogridgen/point_cloud_filtered", 0);
  // pubTest2 = nh_->advertise<pcl::PointCloud<pcl::PointXYZI>>("ogridgen/point_cloud_test", 0);

}

void Classification::findNormals(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud)
{
  if (pointCloud->points.size() < 500)
    return;
  pcl::PointCloud<pcl::Normal>::Ptr normals_out(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
  // norm_est.setSearchMethod(pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>));
  norm_est.setRadiusSearch (.03);
  norm_est.setInputCloud (pointCloud);
  norm_est.setSearchSurface (pointCloud);
  norm_est.compute(*normals_out);

  normals_out->header.frame_id = "map";
  pubTest2.publish(normals_out);
}

// Some segmentation
void Classification::segment(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud) {
  if (pointCloud->points.size() < 1000) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (pointCloud);
  sor.setMeanK (75);
  sor.setStddevMulThresh (.75);
  sor.filter (*cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;


  pcl::PCDWriter writer;


  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // std::stringstream ss;
    // ss << "plane" << i << ".pcd";
    // writer.write<pcl::PointXYZI> (ss.str (), *cloud_p, false);

    pubTest2.publish(cloud_p);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  return;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Classification::filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  if(pointCloud->points.size() < 1) return cloud_filtered;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (pointCloud);
  sor.setMeanK (75);
  sor.setStddevMulThresh (.75);
  sor.filter (*cloud_filtered);
  return cloud_filtered;
}