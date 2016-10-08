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

std::vector<geometry_msgs::Point> FitPlanesToCloud(const navigator_msgs::Buoy &buoy, sensor_msgs::PointCloud &rosCloud) 
{
	std::vector<geometry_msgs::Point> normals;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>), pclCloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud->width = points.size();
	pclCloud->height = 1; //1D data set instead of 2D

	//Downsample while adding to pcl pointcloud
	double voxel = 0.15;
	for (const auto &nextPoint : buoy.points) {
		//if ( fabs(p.z-zo) < height*0.40 ) {
		if ( nextPoint.z > buoy.position.z - buoy.height*0.05 ) {
			pcl::PointXYZ pp;
			pp.x = nextPoint.x; //floor(nextPoint.x/voxel)*voxel+voxel/2;
			pp.y = nextPoint.y; //floor(nextPoint.y/voxel)*voxel+voxel/2;
			pp.z = nextPoint.z; //floor(nextPoint.z/voxel)*voxel+voxel/2;
			pclCloud->push_back(pp);
			rosCloud.points.push_back(nextPoint);
			rosCloud.channels[0].values.push_back(1);
		}
	}
	//Check that we have enough points to run model
	if (buoy.height < 2.5) {
	//if (pclCloud->points.size() < 1000) {
		return normals;
	}

	//Update cloud width 
	pclCloud->width = pclCloud->points.size();

	//Setup plane model
	pcl::ModelCoefficients coeffs;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> segmenter;
	//seg.setOptimizeCoefficients(true);
	segmenter.setEpsAngle(0.25f*M_PI/180.f);
	segmenter.setModelType( pcl:: SACMODEL_PARALLEL_PLANE ); //Documentation is confusing/wrong here... looking for parallel works...
	segmenter.setAxis(Eigen::Vector3f(0,0,1));
	segmenter.setMethodType( pcl::SAC_RANSAC );
	segmenter.setDistanceThreshold(0.15);

  	// Create the filtering object
  	pcl::ExtractIndices<pcl::PointXYZ> extract;

  	//Track maximum indices
  	static size_t maxIndexSize = 0;

	// Segment the largest planar component from the remaining cloud
	segmenter.setInputCloud(pclCloud);
	segmenter.segment(*inliers,coeffs);
	if (inliers->indices.size() < 4) {
  		ROS_INFO_STREAM("PLANE FIT | Danger Will Robinson!");
  		//break;
	} else {
		//Check goodness of fit
		if (inliers->indices.size() > maxIndexSize) { maxIndexSize = inliers->indices.size(); }
		else {
			ROS_INFO_STREAM("PLANE FIT | Previous plane a better match...");
			return normals;
		}

		for (auto ii : inliers->indices) {
			geometry_msgs::Point32 p32;
			p32.x = pclCloud->points[ii].x; p32.y = pclCloud->points[ii].y; p32.z = pclCloud->points[ii].z;
			rosCloud.points.push_back(p32);
			rosCloud.channels[0].values.push_back(100);
		}

    	//Display coeffs
    	ROS_INFO_STREAM("PLANE FIT | Coeffs found: " << coeffs.values[0] << "," << coeffs.values[1] << "," << coeffs.values[2] << " - " << inliers->indices.size());
    	geometry_msgs::Point p;
    	p.x = coeffs.values[0];
    	p.y = coeffs.values[1];
    	p.z = coeffs.values[2];

		//if (fabs(coeffs.values[2]) < 0.5) { 
    		//Rotate the normal 3 times to get the other directions at 90 degree seperation
	    	normals.push_back(p);
	    	for (int ii = 1; ii <= 3; ++ii) {
		    	Eigen::Affine3d RotateZ(Eigen::Affine3d::Identity());
		    	RotateZ.rotate(Eigen::AngleAxisd(M_PI/2.0*ii, Eigen::Vector3d::UnitZ()));
		    	Eigen::Vector3d pRot = RotateZ*Eigen::Vector3d(p.x,p.y,p.z);
		    	geometry_msgs::Point p2;
		    	p2.x = pRot(0); p2.y = pRot(1); p2.z = pRot(2);
		    	ROS_INFO_STREAM("PLANE FIT | Rotated Coeffs found: " << p2.x << "," << p2.y << "," << p2.z);
		    	normals.push_back(p2);
		    }
		    //break;
	    //}
	    
    	// Extract the inliers
    	extract.setInputCloud(pclCloud);
    	extract.setIndices(inliers);
    	extract.setNegative(true);
    	extract.filter(*pclCloud_seg);
    	pclCloud.swap(pclCloud_seg);
	}


	return normals;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									Graveyard
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  	//Run segmenter multiple times
  	int i = 0, nr_points = pclCloud->points.size();
  	// While 30% of the original cloud is still there
  	//while (cloud->points.size () > 0.05 * nr_points)
  	//{




 */