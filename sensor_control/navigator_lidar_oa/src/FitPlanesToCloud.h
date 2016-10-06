#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

std::vector<geometry_msgs::Point> FitPlanesToCloud(const std::vector<geometry_msgs::Point32> points, double height, double zo) {
	std::vector<geometry_msgs::Point> n;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = points.size();
	cloud->height = 1;
	//cloud->points.resize(cloud->width);

	//Downsample while adding to pcl pointcloud
	unsigned ii = 0;
	double voxel = 0.15;
	for (const auto &p : points) {
		//if ( fabs(p.z-zo) < height*0.40 ) {
		if ( p.z > zo-height/2+0.5 ) {
			pcl::PointXYZ pp;
			pp.x = floor(p.x/voxel)*voxel+voxel/2;
			pp.y = floor(p.y/voxel)*voxel+voxel/2;
			pp.z = floor(p.z/voxel)*voxel+voxel/2;
			cloud->push_back(pp);
			++ii;
		}
	}
	cloud->width = ii;

	pcl::ModelCoefficients coeffs;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//seg.setOptimizeCoefficients(true);
	seg.setModelType( pcl:: SACMODEL_PLANE );
	seg.setMethodType( pcl::SAC_RANSAC );
	seg.setDistanceThreshold(0.01);

  	// Create the filtering object
  	pcl::ExtractIndices<pcl::PointXYZ> extract;

  	int i = 0, nr_points = (int) cloud->points.size ();
  	// While 30% of the original cloud is still there
  	while (cloud->points.size () > 0.05 * nr_points)
  	{
    	// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers,coeffs);
    	if (inliers->indices.size() == 0) {
      		ROS_INFO_STREAM("PLANE FIT | Danger Will Robinson!");
      		break;
    	} else {
    		
	    	//Display coeffs
	    	ROS_INFO_STREAM("PLANE FIT | Coeffs found: " << coeffs.values[0] << "," << coeffs.values[1] << "," << coeffs.values[2] << " - " << inliers->indices.size());
	    	geometry_msgs::Point p;
	    	p.x = coeffs.values[0];
	    	p.y = coeffs.values[1];
	    	p.z = coeffs.values[2];

			if (fabs(coeffs.values[2]) < 0.5) { 
		    	n.push_back(p);
		    	for (int ii = 1; ii <= 3; ++ii) {
			    	Eigen::Affine3d RotateZ(Eigen::Affine3d::Identity());
			    	RotateZ.rotate(Eigen::AngleAxisd(M_PI/2.0*ii, Eigen::Vector3d::UnitZ()));
			    	Eigen::Vector3d pRot = RotateZ*Eigen::Vector3d(p.x,p.y,p.z);
			    	geometry_msgs::Point p2;
			    	p2.x = pRot(0); p2.y = pRot(1); p2.z = pRot(2);
			    	ROS_INFO_STREAM("PLANE FIT | Rotated Coeffs found: " << p2.x << "," << p2.y << "," << p2.z);
			    	n.push_back(p2);
			    }
			    break;
		    }
		    
	    	// Extract the inliers
	    	extract.setInputCloud(cloud);
	    	extract.setIndices(inliers);
	    	extract.setNegative(true);
	    	extract.filter(*cloud_f);
	    	cloud.swap(cloud_f);
    	}
	}

	return n;
}