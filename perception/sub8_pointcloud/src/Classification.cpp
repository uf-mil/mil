#include "Classification.hpp"
Classification::Classification(ros::NodeHandle *nh)
{
  nh_ = nh;
  // pubTest = nh_->advertise<visualization_msgs::MarkerArray>( "/test_pub", 0 );
  pubTest = nh_->advertise<pcl::PointCloud<pcl::PointXYZI>>("ogridgen/point_cloud_filtered", 0);
  // pubTest2 = nh_->advertise<pcl::PointCloud<pcl::Normal>>("test_pub/normals", 0);

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

void Classification::segment(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud) {
  if (pointCloud->points.size() < 1000) {
    return;
  }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (pointCloud);
    sor.setMeanK (75);
    sor.setStddevMulThresh (.75);
    sor.filter (*cloud_filtered);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud(pointCloud);
    seg.segment (*inliers, *coefficients);

   pcl::ExtractIndices<pcl::PointXYZI> extract;

   // pcl::PCDWriter writer;
   int i = 0, nr_points = (int) cloud_filtered->points.size ();
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

     // Create the filtering object
     extract.setNegative (true);
     extract.filter (*cloud_f);
     cloud_filtered.swap (cloud_f);
     i++;
   }
     pubTest.publish(cloud_filtered);
  return;
}

void Classification::filtered(pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointCloud) {
  if(pointCloud->points.size() < 1) return; 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (pointCloud);
  sor.setMeanK (75);
  sor.setStddevMulThresh (.75);
  sor.filter (*cloud_filtered);

  cloud_filtered->header.frame_id = "map";
  pubTest.publish(cloud_filtered);

  return;
}