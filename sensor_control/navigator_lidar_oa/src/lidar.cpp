////////////////////////////////////////////////////////////
//
// ROS Lidar Node for RobotX 
//
////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <navigator_msgs/Buoy.h>
#include <navigator_msgs/BuoyArray.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "AStar.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500;
const double ROI_SIZE_METERS = 120;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 20;
const int MAX_HITS_IN_CELL = 500;
const double MAXIMUM_Z_HEIGHT = 12;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS,ROI_SIZE_METERS,VOXEL_SIZE_METERS);
AStar astar(ROI_SIZE_METERS/VOXEL_SIZE_METERS);
nav_msgs::OccupancyGrid rosGrid;
ros::Publisher pubGrid,pubMarkers,pubBuoys;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_velodyne(const sensor_msgs::PointCloud2ConstPtr &pcloud) 
{
	ROS_INFO("cb_velodyne...");	

	//Measure elapsed time for function
	ros::Time timer = ros::Time::now();

	//Use ROS transform listener to grad up-to-date transforms between reference frames
	static tf2_ros::Buffer tfBuffer;
	static tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped T_enu_velodyne_ros;
	try {
		T_enu_velodyne_ros = tfBuffer.lookupTransform("enu", "velodyne",ros::Time(0)); //change time to pcloud header? pcloud->header.stamp 
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }

    //Convert ROS transfrom to eigen transform
    Eigen::Affine3d T_enu_velodyne(Eigen::Affine3d::Identity());
    geometry_msgs::Vector3  lidarpos =  T_enu_velodyne_ros.transform.translation;
    geometry_msgs::Quaternion quat = T_enu_velodyne_ros.transform.rotation;
	T_enu_velodyne.translate(Eigen::Vector3d(lidarpos.x,lidarpos.y,lidarpos.z));
	T_enu_velodyne.rotate(Eigen::Quaterniond(quat.w,quat.x,quat.y,quat.z));
	ROS_INFO_STREAM("Lidar enu: " << lidarpos.x << "," << lidarpos.y << "," << lidarpos.z);

	//Update occupancy grid
	ogrid.setLidarPosition(lidarpos);
	ogrid.updatePointsAsCloud(pcloud,T_enu_velodyne,MAX_HITS_IN_CELL);
	ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY,MAXIMUM_Z_HEIGHT);
	std::vector<objectXYZ> objects;
	std::vector< std::vector<int> > cc = ConnectedComponents(ogrid,objects);
	ogrid.inflateBinary(3);

	//Fake waypoint - this needs to be replaced!
	/*
	Eigen::Vector3d v = T_enu_velodyne*Eigen::Vector3d(30,0,0);;
	int waypoint[3];
	waypoint[0] = (v(0)-lidarpos.x)/VOXEL_SIZE_METERS + ogrid.ROI_SIZE/2;
	waypoint[1] = (v(1)-lidarpos.y)/VOXEL_SIZE_METERS + ogrid.ROI_SIZE/2;
	waypoint[2] = v(2);
	for (int ii = -2; ii <= 2; ++ii) {
		for (int jj = -2; jj <= 2; ++jj) {
			ogrid.ogridMap[ (waypoint[1]+ii)*ogrid.ROI_SIZE + waypoint[0]+jj] = 25;
		}
	}*/

	//Run Astar
	/*
	astar.setMap(ogrid.ogridBinary);
	astar.setFinish(waypoint[0], waypoint[1]);	
	auto solution = astar.run();

	//Set the carrot as the current waypoint position
	geometry_msgs::Pose carrot;
	carrot.position.x = v[0];
	carrot.position.y = v[1];
	carrot.position.z = v[2];

	//If the Astar path still has steps to go, pick a spot a few iterations ahead of the boat
	if (solution.size() >= 15) {
		int index = 0;
		for (auto square : solution) {
			ogrid.ogridMap[square.second*ogrid.ROI_SIZE+square.first] = 0;
			if (index == 15) {
				carrot.position.x = (square.first-ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.x;
				carrot.position.y = (square.second-ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.y;
				//Make the carrot look bigger than it is
				for (int ii = -2; ii <= 2; ++ii) {
					for (int jj = -2; jj <= 2; ++jj) {
						ogrid.ogridMap[ (square.second+ii)*ogrid.ROI_SIZE+square.first+jj] = 75;
					}
				}
			}
			++index;
		}
	}
	ROS_INFO_STREAM("Carrot enu: " << carrot.position.x << "," << carrot.position.y << "," << carrot.position.z);
	*/

	//Publish rosgrid
	rosGrid.header.seq = 0;
	rosGrid.info.resolution = VOXEL_SIZE_METERS;
	rosGrid.header.frame_id = "enu";
	rosGrid.header.stamp = ros::Time::now();
	rosGrid.info.map_load_time = ros::Time::now();
	rosGrid.info.width = ogrid.ROI_SIZE;
	rosGrid.info.height = ogrid.ROI_SIZE;
	rosGrid.info.origin.position.x = ogrid.lidarPos.x + ogrid.ROItoMeters(0);
	rosGrid.info.origin.position.y = ogrid.lidarPos.y + ogrid.ROItoMeters(0);
	rosGrid.info.origin.position.z = ogrid.lidarPos.z;
	rosGrid.data = ogrid.ogridMap;
	pubGrid.publish(rosGrid);

	//Publish markers
	geometry_msgs::Point p;
	visualization_msgs::MarkerArray markers;	
	visualization_msgs::Marker m;
	m.header.stamp = ros::Time::now();
	m.header.seq = 0;
	m.header.frame_id = "enu";
	
	//Erase old markers
	m.id = 0;
	m.type = 0;
	m.action = 3;
	markers.markers.push_back(m);

	//Course Outline
	m.id = 1;
	m.type = visualization_msgs::Marker::LINE_STRIP;
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = 0.5;
	p.x = -35; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
	p.x = -35; p.y = -35; p.z = lidarpos.z; m.points.push_back(p);
	p.x = 65; p.y = -35; p.z = lidarpos.z; m.points.push_back(p);
	p.x = 65; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
	p.x = -35; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
	m.color.a = 0.6; m.color.r = 1; m.color.g = 1; m.color.b = 1;
	markers.markers.push_back(m);
	
	//Publish buoys
	navigator_msgs::BuoyArray allBuoys;
	navigator_msgs::Buoy buoy;
	geometry_msgs::Point32 p32;
	buoy.header.seq = 0;
	buoy.header.frame_id = "enu";
	buoy.header.stamp = ros::Time::now();	
	int id = 0;
	for (auto obj : objects) {
		if (obj.position.x < -35 || obj.position.x > 65 || obj.position.y < -35 || obj.position.y > 50 ) { continue; }
		ROS_INFO_STREAM("Adding buoy " << id << " at " << obj.position.x << "," << obj.position.y << "," << obj.position.z);
		//Buoys
		buoy.header.stamp = ros::Time::now();
		buoy.id = id;
		buoy.confidence = 0;
		buoy.position = obj.position;
		buoy.height = obj.scale.z; 
		buoy.width = obj.scale.x; //x or y for width?
		allBuoys.buoys.push_back(buoy);

		//Buoys as markers
		visualization_msgs::Marker m2;
		m2.header.stamp = ros::Time::now();
		m2.header.seq = 0;
		m2.header.frame_id = "enu";		
		m2.header.stamp = ros::Time::now();
		m2.id = id+2;
		m2.type = visualization_msgs::Marker::CUBE;
		m2.action = visualization_msgs::Marker::ADD;		
		m2.pose.position = obj.position;
		m2.scale = obj.scale;
		m2.color.a = 0.6; m2.color.r = 1; m2.color.g = 1; m2.color.b = 1;
		markers.markers.push_back(m2);
		++id;
	}
	pubMarkers.publish(markers);
	pubBuoys.publish(allBuoys);

	//Elapsed time
	ROS_INFO_STREAM("Elapsed time: " << (ros::Time::now()-timer).toSec());
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update odometry information
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_odom(const nav_msgs::OdometryConstPtr &odom) {
	//ROS_INFO("cb_odom...");	
	boatPose_enu = odom->pose.pose;
	boatTwist_enu = odom->twist.twist;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	//Ros init
	ros::init(argc, argv, "lidar");
	ros::Time::init();
	//ros::init(argc, argv, "lidar", ros::init_options::AnonymousName);

	//Check that ROS is alive before continuing... After 10 minutes quit!
	ROS_INFO("Checking ROS master is alive...");
	ros::Time timer = ros::Time::now();
	while (!ros::master::check()) {
		if ( (ros::Time::now()-timer).toSec() > 600) { return -1; }
		ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("ROS Master: " << ros::master::getHost());

	//Initialize Astar to use 8 way search method
	astar.setMode(AStar::ASTAR,AStar::EIGHT);
	
	//Node handler
	ros::NodeHandle nh;

	//Subscribe to odom and the velodyne
	ros::Subscriber sub1 = nh.subscribe("/velodyne_points", 1, cb_velodyne);
	ros::Subscriber sub2 = nh.subscribe("/odom", 1, cb_odom);

	//Publish occupancy grid and visualization markers
	pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("ogrid_batcave",10);
	pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("markers_batcave",10);
	pubBuoys = nh.advertise<navigator_msgs::BuoyArray>("buoys_batcave",10);

	//Give control to ROS
	ros::spin();
	//ros::spinOnce();

	return 0;
}











































/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									Graveyard
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	//Get nodes and topics
	ros::V_string nodes;
	ros::master::getNodes(nodes);
	cout << "*** Nodes ***" << endl;
	for (auto ii : nodes) { cout << ii << endl; }

	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);
	cout << "*** Topics ***" << endl;
	for (auto ii : topics) { cout << ii.name << " (" << ii.datatype << ")" << endl; }
	*/

/*

		//ROS_INFO_STREAM("T " << T.matrix());
		//pcl_ros::transformPointCloud2(*pcloud, pcloudWorld,T_fixedToVelodyne);
    	//tfListen.lookupTransform("/base_link", "/enu",ros::Time(0), T_fixedToVelodyne);
    	//tfListen.transformPointCloud("enu",*pcloud,pcloudWorld);
    	//tf2::doTransform (*pcloud, pcloudWorld, T_fixedToVelodyne);

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include "pcl_ros/point_cloud.h"

//ROS message filter needed?
void cb_transform(const tf2_msgs::TFMessageConstPtr &transform ) 
{
	ROS_INFO("cb_transform...");
	Eigen::Affine3d T_F_B(Eigen::Affine3d::Identity());
	for (int ii = 0; ii < transform->transforms.size(); ++ii) {
		geometry_msgs::TransformStamped tStamped = transform->transforms[ii];
		ROS_INFO_STREAM(tStamped.header.frame_id << " to " << tStamped.child_frame_id);
	}
		//T_F_B = tf2::transformToEigen (tStamped);
}

	//Get one message
	boost::shared_ptr<const sensor_msgs::PointCloud2> cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points",ros::Duration(10));
	ROS_INFO_STREAM("Cloud height/width: " << cloud->height << "," << cloud->width << "," << cloud->header.frame_id);
	ROS_INFO_STREAM("Cloud point-step/row-step: " << cloud->point_step << "," << cloud->row_step<< "," << (int)cloud->is_bigendian);
	for (auto ii = 0; ii < 4; ++ii) {
		ROS_INFO_STREAM("Cloud field " << ii << ": " << cloud->fields[ii].name << "," << cloud->fields[ii].offset << "," << (int)cloud->fields[ii].datatype << "," << cloud->fields[ii].count);
	}
	
	
	//Raw Step through data
	for (auto ii = 0, jj = 0; ii < cloud->width; ++ii,jj+=cloud->point_step) {
		floatConverter x,y,z,i;
		for (int kk = 0; kk < 4; ++kk)
		{
			x.data[kk] = cloud->data[jj+kk];
			y.data[kk] = cloud->data[jj+4+kk];
			z.data[kk] = cloud->data[jj+8+kk];
			i.data[kk] = cloud->data[jj+16+kk];
		}
		std::cout << -y.f << "\t" << z.f << "\t" << -x.f << "\t" << i.f << std::endl;
	}
	
	
	//Use PCL to convert data to XYZI
	pcl::PointCloud<pcl::PointXYZI> pclCloud;
	pcl::fromROSMsg(*cloud,pclCloud);
	//Save point cloud to file - Switch order for OpenGL
	for (size_t i = 0; i < pclCloud.points.size(); ++i) {
    	std::cout << -pclCloud.points[i].y << "\t" << pclCloud.points[i].z << "\t" << -pclCloud.points[i].x << "\t" << pclCloud.points[i].intensity << std::endl;
    }

    */
//********************************************************************************************************************************
//********************************************************************************************************************************
//********************************************************************************************************************************
//********************************************************************************************************************************
