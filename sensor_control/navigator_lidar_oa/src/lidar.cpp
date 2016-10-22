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
#include <navigator_msgs/PerceptionObject.h>
#include <navigator_msgs/PerceptionObjectArray.h>
#include <navigator_msgs/ObjectDBQuery.h>
#include <navigator_msgs/Bounds.h>
#include <uf_common/PoseTwistStamped.h>

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "objects.h"
#include "bounding_boxes.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500;
const double ROI_SIZE_METERS = 201;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 25; //20
const int MAX_HITS_IN_CELL = 125; //500
const double MAXIMUM_Z_BELOW_LIDAR = 2; //2
const double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
const double MAX_ROLL_PITCH_ANGLE_DEG = 5.3;
const double LIDAR_VIEW_ANGLE_DEG = 70;
const double LIDAR_VIEW_DISTANCE_METERS = 35;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS,ROI_SIZE_METERS,VOXEL_SIZE_METERS);
nav_msgs::OccupancyGrid rosGrid;
ros::Publisher pubGrid,pubMarkers,pubObjects,pubCloudPersist,pubCloudFrame,pubCloudPCL;
ObjectTracker object_tracker;
geometry_msgs::Point waypoint_ogrid;
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
uf_common::PoseTwistStamped waypoint_enu,carrot_enu;
ros::Time pubObjectsTimer;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These are changed on startup if /get_bounds service is present
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d BOUNDARY_CORNER_1 (30, 10);
Eigen::Vector2d BOUNDARY_CORNER_2 (30, 120);
Eigen::Vector2d BOUNDARY_CORNER_3 (140, 120);
Eigen::Vector2d BOUNDARY_CORNER_4 (140, 10);

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


	//Skip lidar updates if roll or pitch is too high 
	/*
	Eigen::Matrix3d mat = R_enu_velodyne.rotation();
	double ang[3];
	ang[0] = atan2(-mat(1,2), mat(2,2) );    
   	double sr = sin( ang[0] ), cr = cos( ang[0] );
   	ang[1] = atan2( mat(0,2),  cr*mat(2,2) - sr*mat(1,2) );
   	ang[2] = atan2( -mat(0,1), mat(0,0) ); 
	ROS_INFO_STREAM("LIDAR | Velodyne enu: " << lidarPos.x << "," << lidarPos.y << "," << lidarPos.z);
	ROS_INFO_STREAM("LIDAR | XYZ Rotation: " << ang[0]*180/M_PI << "," << ang[1]*180/M_PI << "," << ang[2]*180/M_PI );	
	if (fabs(ang[0]*180/M_PI) > MAX_ROLL_PITCH_ANGLE_DEG || fabs(ang[1]*180/M_PI) > MAX_ROLL_PITCH_ANGLE_DEG) {
		return;
	}
	*/

	//Set bounding box
	ogrid.setBoundingBox(BOUNDARY_CORNER_1,BOUNDARY_CORNER_2,BOUNDARY_CORNER_3,BOUNDARY_CORNER_4);

	//Update occupancy grid
	ogrid.setLidarPosition(lidarPos,lidarHeading);
	ogrid.setLidarParams(LIDAR_VIEW_ANGLE_DEG,LIDAR_VIEW_DISTANCE_METERS);
	ogrid.updatePointsAsCloud(pcloud,T_enu_velodyne,MAX_HITS_IN_CELL,MAXIMUM_Z_BELOW_LIDAR,MAXIMUM_Z_ABOVE_LIDAR);
	ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY);

	//Inflate ogrid before detecting objects and calling AStar
	ogrid.inflateBinary(2);

	//Detect objects
	std::vector<objectMessage> objects;
	std::vector< std::vector<int> > cc = ConnectedComponents(ogrid,objects);


	//Publish second point cloud
	sensor_msgs::PointCloud objectCloudPersist,objectCloudFrame,pclCloud;
	objectCloudPersist.header.seq = 0;
	objectCloudPersist.header.frame_id = "enu";
	objectCloudPersist.header.stamp = ros::Time::now();
	objectCloudFrame.header.seq = 0;
	objectCloudFrame.header.frame_id = "enu";
	objectCloudFrame.header.stamp = ros::Time::now();	
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
	//std::cout << ogrid.lidarPos.x << " vs " << (ogrid.boatCol-ogrid.GRID_SIZE/2)*VOXEL_SIZE_METERS  << endl;
	rosGrid.info.origin.position.x = (ogrid.boatCol-ogrid.GRID_SIZE/2)*VOXEL_SIZE_METERS + ogrid.ROItoMeters(0);//ogrid.lidarPos.x + ogrid.ROItoMeters(0);
	rosGrid.info.origin.position.y = (ogrid.boatRow-ogrid.GRID_SIZE/2)*VOXEL_SIZE_METERS + ogrid.ROItoMeters(0);//ogrid.lidarPos.y + ogrid.ROItoMeters(0);
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
	p.x = lidarPos.x; p.y = lidarPos.y; p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
	m.points.push_back(p);	
	for (double theta = -LIDAR_VIEW_ANGLE_DEG*M_PI/180.0; theta <= LIDAR_VIEW_ANGLE_DEG*M_PI/180.0; theta += 0.1) {
		Eigen::Affine3d RotateZ(Eigen::Affine3d::Identity());
	    RotateZ.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())); 
	    Eigen::Vector3d heading = RotateZ*lidarHeading;
	    p.x = heading(0)*LIDAR_VIEW_DISTANCE_METERS+lidarPos.x; p.y = heading(1)*LIDAR_VIEW_DISTANCE_METERS+lidarPos.y; p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR; 
		m.points.push_back(p);
	}
	p.x = lidarPos.x; p.y = lidarPos.y; p.z = lidarPos.z - MAXIMUM_Z_BELOW_LIDAR;  
	m.points.push_back(p);
	markers.markers.push_back(m);	
	geometry_msgs::Point32 p32;	
	sensor_msgs::ChannelFloat32 channel;
	channel.name = "intensity";
	objectCloudPersist.channels.push_back(channel);
	objectCloudFrame.channels.push_back(channel);
	pclCloud.channels.push_back(channel);
	
	auto object_permanence = object_tracker.add_objects(objects,pclCloud,boatPose_enu);

	int max_id = 0;	
	for (auto obj : object_permanence) {
		ROS_INFO_STREAM("LIDAR | " << obj.name << " " << obj.id << " at " << obj.position.x << "," << obj.position.y << "," << obj.position.z << " with " << obj.strikesPersist.size() << "(" << obj.strikesFrame.size() << ") points, size " << obj.scale.x << "," << obj.scale.y << "," << obj.scale.z << " maxHeight " << obj.maxHeightFromLidar);

		//Show point cloud of just objects
		objectCloudPersist.points.insert(objectCloudPersist.points.end(),obj.strikesPersist.begin(),obj.strikesPersist.end());
		for (auto ii : obj.intensityPersist) {	
			objectCloudPersist.channels[0].values.push_back(ii);
		}

		//Show point cloud of just objects
		objectCloudFrame.points.insert(objectCloudFrame.points.end(),obj.strikesFrame.begin(),obj.strikesFrame.end());
		for (auto ii : obj.intensityFrame) {	
			objectCloudFrame.channels[0].values.push_back(ii);
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
		m4.pose.position.z += 3.5;
		m4.scale.z = 2;
		m4.text = to_string(obj.id) + ":" + obj.name.substr(0,2);
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
	pubCloudPersist.publish(objectCloudPersist);
	pubCloudFrame.publish(objectCloudFrame);
	pubCloudPCL.publish(pclCloud);

	//Publish PerceptionObjects at some slower rate
	if ( (ros::Time::now() - pubObjectsTimer).toSec() > 2 ) {
		navigator_msgs::PerceptionObjectArray objectArray;
		object_tracker.lookUpByName("all",objectArray.objects);
		pubObjects.publish(objectArray);
		pubObjectsTimer = ros::Time::now();
	}
	

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
//Handle DB Requests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool objectRequest(navigator_msgs::ObjectDBQuery::Request  &req, navigator_msgs::ObjectDBQuery::Response &res) 
{
	ROS_INFO_STREAM("LIDAR | DB request with name " << req.name << " and command " << req.cmd);
	res.found = false;
	res.objects.clear();

	//Did we get a command?
	//Each command needs an id and value seperated by equals sign, this is ugly code...
	auto index = req.cmd.find('=');
	if (index != std::string::npos) {
		auto id = stoi(req.cmd.substr(0,index));
		//Is the second part a name or color values?
		auto comma1 = req.cmd.find(',',index);
		auto comma2 = req.cmd.find(',',comma1+1);
		if (comma1 == string::npos || comma2 == string::npos) {
			auto name = req.cmd.substr(index+1);
			if (name == "shooter" || name == "dock" || name == "scan_the_code" || name == "totem" || name == "start_gate" || name == "unknown") {
				for (auto &obj : object_tracker.saved_objects) {
					if (obj.id == id) { 
						ROS_INFO_STREAM("LIDAR | Changing id of object " << id << " to " << name);
						obj.name = name; break; 
					}
				}
			}
		} else {
			auto r = stoi( req.cmd.substr(index+1,comma1-index-1) );
			auto g = stoi( req.cmd.substr(comma1+1,comma2-comma1-1) );
			auto b = stoi( req.cmd.substr(comma2+1) );
			
			for (auto &obj : object_tracker.saved_objects) {
				if (obj.id == id) { 
					obj.color.r = r; obj.color.g = g; obj.color.b = b; 
					ROS_INFO_STREAM("LIDAR | Changing color of object " << id << " to " << r << "," << g << "," << b);
					break; 
				}
			}			

		}
		return true;
	}

	//My original object message doesn't exactly match PerceptionObject, fix this in future!
	res.found = object_tracker.lookUpByName(req.name,res.objects);

	return true;
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
	ROS_INFO_STREAM("LIDAR | ROS Master: " << ros::master::getHost());

	//Node handler
	ros::NodeHandle nh;

	//Subscribe to odom and the velodyne
	ros::Subscriber sub1 = nh.subscribe("/velodyne_points", 1, cb_velodyne);
	ros::Subscriber sub2 = nh.subscribe("/odom", 1, cb_odom);

	//Publish occupancy grid and visualization markers
	pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("/ogrid",10);
	pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/unclassified_markers",10);

	//Publish PerceptionObjects
	pubObjects = nh.advertise<navigator_msgs::PerceptionObjectArray>("/database/objects",1);
	pubObjectsTimer = ros::Time::now();

	//Extra publishing for debugging...
	pubCloudPersist = nh.advertise<sensor_msgs::PointCloud>("ira_persist",1);
	pubCloudFrame = nh.advertise<sensor_msgs::PointCloud>("ira_frame",1);
	pubCloudPCL = nh.advertise<sensor_msgs::PointCloud>("ira_pclcloud",1);

	//Service for object request
	ros::ServiceServer service = nh.advertiseService("/database/requests", objectRequest);

	//Check for bounds from parameter server on startup
	ros::ServiceClient boundsClient = nh.serviceClient<navigator_msgs::Bounds>("/get_bounds");
	navigator_msgs::Bounds::Request boundsReq;
	navigator_msgs::Bounds::Response boundsRes;
	boundsReq.to_frame = "enu";
	if (boundsClient.call(boundsReq,boundsRes) && boundsRes.bounds.size() == 4) {
		BOUNDARY_CORNER_1(0) = boundsRes.bounds[0].x; BOUNDARY_CORNER_1(1) = boundsRes.bounds[0].y;
		BOUNDARY_CORNER_2(0) = boundsRes.bounds[1].x; BOUNDARY_CORNER_2(1) = boundsRes.bounds[1].y;
		BOUNDARY_CORNER_3(0) = boundsRes.bounds[2].x; BOUNDARY_CORNER_3(1) = boundsRes.bounds[2].y;
		BOUNDARY_CORNER_4(0) = boundsRes.bounds[3].x; BOUNDARY_CORNER_4(1) = boundsRes.bounds[3].y;
		ROS_INFO_STREAM("LIDAR | Bounds set to rosparam");
	} else {
		ROS_INFO_STREAM("LIDAR | Bounds not available from /get_bounds service....");
	}

	//Give control to ROS
	ros::spin();

	return 0;
}
