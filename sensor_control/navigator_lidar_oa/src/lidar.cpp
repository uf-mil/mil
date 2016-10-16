////////////////////////////////////////////////////////////
//
// ROS Lidar Node for RobotX 
//
////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <navigator_msgs/Buoy.h>
#include <navigator_msgs/BuoyArray.h>
#include <uf_common/PoseTwistStamped.h>
#include <uf_common/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "AStar.h"
#include "objects.h"
#include "bounding_boxes.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500;
const double ROI_SIZE_METERS = 201;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 25; //20
const int MAX_HITS_IN_CELL = 100; //500
const double MAXIMUM_Z_BELOW_LIDAR = 2; //2
const double MAXIMUM_Z_ABOVE_LIDAR = 2;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS,ROI_SIZE_METERS,VOXEL_SIZE_METERS);
AStar astar(ROI_SIZE_METERS/VOXEL_SIZE_METERS);
nav_msgs::OccupancyGrid rosGrid;
ros::Publisher pubGrid,pubMarkers,pubBuoys,pubTrajectory,pubWaypoint,pubCloud,pubCloudPCL;
ObjectTracker object_tracker;
geometry_msgs::Point waypoint_ogrid;
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
uf_common::PoseTwistStamped waypoint_enu,carrot_enu;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d BOUNDARY_CORNER_1 (30, 10);
Eigen::Vector2d BOUNDARY_CORNER_2 (30, 120);
Eigen::Vector2d BOUNDARY_CORNER_3 (140, 120);
Eigen::Vector2d BOUNDARY_CORNER_4 (140, 10);

//Eigen::Vector2d BOUNDARY_CORNER_1 (-30-35, 50+20);
//Eigen::Vector2d BOUNDARY_CORNER_2 (-30-35, -20+20);
//Eigen::Vector2d BOUNDARY_CORNER_3 (35-35, -20+20);
//Eigen::Vector2d BOUNDARY_CORNER_4 (35-35, 50+20);

//Eigen::Vector2d BOUNDARY_CORNER_1 (-30, 50);
//Eigen::Vector2d BOUNDARY_CORNER_2 (-30, -20);
//Eigen::Vector2d BOUNDARY_CORNER_3 (35, -20);
//Eigen::Vector2d BOUNDARY_CORNER_4 (35, 50);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_velodyne(const sensor_msgs::PointCloud2ConstPtr &pcloud) 
{
	ROS_INFO("**********************************************************");
	ROS_INFO("LIDAR | cb_velodyne...");	

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

    //Convert ROS transform to eigen transform
    Eigen::Affine3d T_enu_velodyne(Eigen::Affine3d::Identity());
    Eigen::Affine3d R_enu_velodyne(Eigen::Affine3d::Identity());
    geometry_msgs::Vector3  lidarPos =  T_enu_velodyne_ros.transform.translation;
    geometry_msgs::Quaternion quat = T_enu_velodyne_ros.transform.rotation;
	T_enu_velodyne.translate(Eigen::Vector3d(lidarPos.x,lidarPos.y,lidarPos.z));
	T_enu_velodyne.rotate(Eigen::Quaterniond(quat.w,quat.x,quat.y,quat.z));
	R_enu_velodyne.rotate(Eigen::Quaterniond(quat.w,quat.x,quat.y,quat.z));
	Eigen::Vector3d lidarHeading = R_enu_velodyne*Eigen::Vector3d(1,0,0);
	ROS_INFO_STREAM("LIDAR | Velodyne enu: " << lidarPos.x << "," << lidarPos.y << "," << lidarPos.z);

	//Set bounding box
	ogrid.setBoundingBox(BOUNDARY_CORNER_1,BOUNDARY_CORNER_2,BOUNDARY_CORNER_3,BOUNDARY_CORNER_4);

	//Update occupancy grid
	ogrid.setLidarPosition(lidarPos,lidarHeading);
	ogrid.updatePointsAsCloud(pcloud,T_enu_velodyne,MAX_HITS_IN_CELL,MAXIMUM_Z_BELOW_LIDAR,MAXIMUM_Z_ABOVE_LIDAR);
	ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY);

	//Inflate ogrid before detecting objects and calling AStar
	ogrid.inflateBinary(2);

	//Detect objects
	std::vector<objectMessage> objects;
	std::vector< std::vector<int> > cc = ConnectedComponents(ogrid,objects);


	//Publish second point cloud
	sensor_msgs::PointCloud buoyCloud,pclCloud;
	buoyCloud.header.seq = 0;
	buoyCloud.header.frame_id = "enu";
	buoyCloud.header.stamp = ros::Time::now();
	pclCloud.header.seq = 0;
	pclCloud.header.frame_id = "enu";
	pclCloud.header.stamp = ros::Time::now();

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
	rosGrid.info.origin.position.z = ogrid.lidarPos.z - MAXIMUM_Z_BELOW_LIDAR;
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
	m.id = 1000;
	m.type = 0;
	m.action = 3;
	markers.markers.push_back(m);

	//Course Outline - change to real values or pull from service/topic
	m.id = 1001;
	m.type = visualization_msgs::Marker::LINE_STRIP;
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = 0.5;
	p.x = BOUNDARY_CORNER_1(0); p.y = BOUNDARY_CORNER_1(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = BOUNDARY_CORNER_2(0); p.y = BOUNDARY_CORNER_2(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = BOUNDARY_CORNER_3(0); p.y = BOUNDARY_CORNER_3(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = BOUNDARY_CORNER_4(0); p.y = BOUNDARY_CORNER_4(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = BOUNDARY_CORNER_1(0); p.y = BOUNDARY_CORNER_1(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	m.color.a = 0.6; m.color.r = 1; m.color.g = 1; m.color.b = 1;
	markers.markers.push_back(m);

	//Lidar area
	m.id = 1002;
	m.points.clear();
	Eigen::Vector3d lidarLeft = T_enu_velodyne*Eigen::Vector3d(0,100,0);
	p.x = lidarLeft(0); p.y = lidarLeft(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	Eigen::Vector3d lidarUpLeft = T_enu_velodyne*Eigen::Vector3d(100,100,0);
	p.x = lidarUpLeft(0); p.y = lidarUpLeft(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	Eigen::Vector3d lidarUpRight = T_enu_velodyne*Eigen::Vector3d(100,-100,0);
	p.x = lidarUpRight(0); p.y = lidarUpRight(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	Eigen::Vector3d lidarRight = T_enu_velodyne*Eigen::Vector3d(0,-100,0);
	p.x = lidarRight(0); p.y = lidarRight(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = lidarPos.x; p.y = lidarPos.y; p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	p.x = lidarLeft(0); p.y = lidarLeft(1); p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);
	m.color.a = 0.6; m.color.r = 1; m.color.g = 0; m.color.b = 0;
	markers.markers.push_back(m);
	
	//Publish buoys
	//navigator_msgs::BuoyArray allBuoys;
	//navigator_msgs::Buoy buoy;
	geometry_msgs::Point32 p32;
	
	sensor_msgs::ChannelFloat32 channel;
	channel.name = "intensity";
	buoyCloud.channels.push_back(channel);
	pclCloud.channels.push_back(channel);
	//buoy.header.seq = 0;
	//buoy.header.frame_id = "enu";
	//buoy.header.stamp = ros::Time::now();	
	

	auto object_permanence = object_tracker.add_objects(objects,pclCloud,boatPose_enu);

	//Doesn't seem like small_objects being used, so commented...
	//std::vector<objectMessage> small_objects = BoundingBox::get_accurate_objects(pcloud, object_permanence, T_enu_velodyne);

	int max_id = 0;	
	for (auto obj : object_permanence) {
		ROS_INFO_STREAM("LIDAR | Buoy " << obj.id << " at " << obj.position.x << "," << obj.position.y << "," << obj.position.z << " with " << obj.beams.size() << " points and size " << obj.scale.x << "," << obj.scale.y << "," << obj.scale.z);
		
		//Convert obj to buoy ros message
		/*buoy.header.stamp = ros::Time::now();
		buoy.id = obj.id;
		buoy.confidence = 0;
		buoy.position = obj.position;
		buoy.height = obj.scale.z; 
		buoy.width = obj.scale.x; 
		buoy.depth = obj.scale.y; 
		buoy.points = obj.beams;
		buoy.intensity = obj.intensity;
		buoy.pclInliers = obj.pclInliers;
		buoy.normal = obj.normal;*/

		//Show point cloud of just buoy objects
		buoyCloud.points.insert(buoyCloud.points.end(),obj.beams.begin(),obj.beams.end());
		for (auto ii : obj.intensity) {	
			buoyCloud.channels[0].values.push_back(ii);
		}

		//Display number next to object
		visualization_msgs::Marker m4;
		m4.header.stamp = ros::Time::now();
		m4.header.seq = 0;
		m4.header.frame_id = "enu";		
		m4.header.stamp = ros::Time::now();
		m4.id = obj.id+3000;
		m4.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		m4.action = visualization_msgs::Marker::ADD;
		m4.pose.position = obj.position;
		m4.pose.position.x += 1.5;
		m4.scale.z = 2.5;
		m4.text = to_string(obj.id);
		m4.color.a = 0.6; m4.color.r = 0; m4.color.g = 1; m4.color.b = 0;		
		markers.markers.push_back(m4);

		//Display normal as an arrow
		if (obj.pclInliers > 10) {
			visualization_msgs::Marker m3;
			m3.header.stamp = ros::Time::now();
			m3.header.seq = 0;
			m3.header.frame_id = "enu";		
			m3.header.stamp = ros::Time::now();
			m3.id = obj.id+2000;
			m3.type = visualization_msgs::Marker::ARROW;
			m3.action = visualization_msgs::Marker::ADD;
			m3.points.push_back(obj.position);
			geometry_msgs::Point pp;
			pp.x = obj.position.x+obj.normal.x*5; pp.y = obj.position.y+obj.normal.y*5; pp.z = obj.position.z+obj.normal.z*5;		
			m3.points.push_back(pp);
			m3.scale.x = 1; m3.scale.y = 1; m3.scale.z = 1;
			m3.color.a = 0.6; m3.color.r = 1; m3.color.g = 1; m3.color.b = 1;
			markers.markers.push_back(m3);
		}

		//Push buoy into collection
		//allBuoys.buoys.push_back(buoy);

		//Turn obj into a marker for rviz
		visualization_msgs::Marker m2;
		m2.header.stamp = ros::Time::now();
		m2.header.seq = 0;
		m2.header.frame_id = "enu";		
		m2.header.stamp = ros::Time::now();
		m2.id = obj.id;
		m2.type = visualization_msgs::Marker::CUBE;
		m2.action = visualization_msgs::Marker::ADD;		
		m2.pose.position = obj.position;
		m2.scale = obj.scale;
		m2.color.a = 0.6; m2.color.r = 1; m2.color.g = 1; m2.color.b = 1;
		markers.markers.push_back(m2);
		if(m2.id > max_id) max_id = m2.id;
	}	
	pubMarkers.publish(markers);
	//pubBuoys.publish(allBuoys);
	pubCloud.publish(buoyCloud);
	pubCloudPCL.publish(pclCloud);


	//std::cout<<"MAX: "<<max_id<<std::endl;
	// small_markers.markers.clear();
	// m.header.seq = 0;
	// m.header.frame_id = "enu";
	// m.action = 3;
	// for (auto obj : small_objects) {
	// 	if (obj.scale.x > 10 || obj.scale.y > 10 || obj.scale.z > 10) { continue; }
	// 	m.header.stamp = ros::Time::now();
	// 	m.id = obj.id;
	// 	m.type = visualization_msgs::Marker::CUBE;
	// 	m.action = visualization_msgs::Marker::ADD;
	// 	m.pose.position = obj.position;
	// 	m.scale = obj.scale;
	// 	m.color.a = 0.6; m.color.r = 1; m.color.g = 1; m.color.b = 1;
	// 	small_markers.markers.push_back(m);
	// 	++id;
	// }
	// pubMarkersSmall.publish(small_markers);

	//Elapsed time
	ROS_INFO_STREAM("LIDAR | Elapsed time: " << (ros::Time::now()-timer).toSec());
	ROS_INFO("**********************************************************");
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
	ROS_INFO("LIDAR | Checking ROS master is alive...");
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
	pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("ira_ogrid",10);
	pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("ira_markers",10);
	//pubBuoys = nh.advertise<navigator_msgs::BuoyArray>("buoys_batcave",10);
	//pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/unclassified/objects/markers",10);
	//pubBuoys = nh.advertise<navigator_msgs::BuoyArray>("/unclassified/objects",10);	
	pubCloud = nh.advertise<sensor_msgs::PointCloud>("ira_cloud",1);
	pubCloudPCL = nh.advertise<sensor_msgs::PointCloud>("ira_pclcloud",1);

	//Give control to ROS
	ros::spin();

	return 0;
}