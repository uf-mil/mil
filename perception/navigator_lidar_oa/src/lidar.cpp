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
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <uf_common/PoseTwistStamped.h>

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "objects.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500;
const double ROI_SIZE_METERS = 201;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 25; //20
const int MAX_HITS_IN_CELL = 125; //500
const double MAXIMUM_Z_BELOW_LIDAR = 2; //2
const double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
const double MAX_ROLL_PITCH_ANGLE_DEG = 5.3;
const double LIDAR_VIEW_ANGLE_DEG = 160;
const double LIDAR_VIEW_DISTANCE_METERS = 60;
const double LIDAR_MIN_VIEW_DISTANCE_METERS = 5.5;
const int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
const double MIN_OBJECT_HEIGHT_METERS = 0.075;
const double MIN_OBJECT_SEPERATION_DISTANCE = 1.5;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Random collection of globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS,ROI_SIZE_METERS,VOXEL_SIZE_METERS);
nav_msgs::OccupancyGrid rosGrid;
ros::Publisher pubGrid,pubMarkers,pubObjects,pubCloudPersist,pubCloudFrame,pubCloudPCL;
ros::ServiceClient boundsClient;
ObjectTracker object_tracker(MIN_OBJECT_SEPERATION_DISTANCE*3);
geometry_msgs::Point waypoint_ogrid;
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
uf_common::PoseTwistStamped waypoint_enu,carrot_enu;
ros::Time pubObjectsTimer;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interactive marker globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
interactive_markers::InteractiveMarkerServer *markerServer;
interactive_markers::MenuHandler menuHandler;
interactive_markers::MenuHandler::EntryHandle menuEntry;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These are changed on startup if /get_bounds service is present
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Eigen::Vector2d BOUNDARY_CORNER_1 (30-80, 10-150);
//Eigen::Vector2d BOUNDARY_CORNER_2 (30-80, 120-150);
//Eigen::Vector2d BOUNDARY_CORNER_3 (140-80, 120-150);
//Eigen::Vector2d BOUNDARY_CORNER_4 (140-80, 10-150);

Eigen::Vector2d BOUNDARY_CORNER_1 (0, 0);
Eigen::Vector2d BOUNDARY_CORNER_2 (1, 0);
Eigen::Vector2d BOUNDARY_CORNER_3 (1, -1);
Eigen::Vector2d BOUNDARY_CORNER_4 (0, -1);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Helper function for making interactive markers
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
visualization_msgs::InteractiveMarker CreateInteractiveMarker(string name)
{
		//Turn obj into an interactive marker for rviz
	  	visualization_msgs::InteractiveMarker int_marker; 
	  	visualization_msgs::InteractiveMarkerControl control;
	  	int_marker.header.frame_id = "enu";
	  	int_marker.scale = 1;
	  	int_marker.name = name;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	  	control.always_visible = true;
	  	control.name = name;
	  	int_marker.controls.push_back(control);
	  	return int_marker;
}

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

	//Check for bounds from parameter server on startup
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
	ogrid.setLidarParams(LIDAR_VIEW_ANGLE_DEG,LIDAR_VIEW_DISTANCE_METERS,LIDAR_MIN_VIEW_DISTANCE_METERS,MIN_LIDAR_POINTS_FOR_OCCUPANCY,MIN_OBJECT_HEIGHT_METERS);
	ogrid.updatePointsAsCloud(pcloud,T_enu_velodyne,MAX_HITS_IN_CELL,MAXIMUM_Z_BELOW_LIDAR,MAXIMUM_Z_ABOVE_LIDAR);
	ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY);

	//Inflate ogrid before detecting objects and calling AStar
	ogrid.inflateBinary(2);

	//Detect objects
	std::vector<objectMessage> objects;
	std::vector< std::vector<int> > cc = ConnectedComponents(ogrid,objects,MIN_OBJECT_SEPERATION_DISTANCE);


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
	
	//Track objects over frames
	auto object_permanence = object_tracker.add_objects(objects,pclCloud,boatPose_enu);

	//Look for gates in between 4 buoys
	auto gatePosAndNormal = object_tracker.FindThreeGates();
	auto gatePositions = std::get<0>(gatePosAndNormal);
	auto gateNormal = std::get<1>(gatePosAndNormal);
	std::vector<std::string> names{"Gate_2","Gate_1","Gate_3"};
	for (auto ii = 0; ii < gatePositions.size(); ++ii) {
		geometry_msgs::Pose newPose;
		newPose.position = gatePositions[ii];
		markerServer->setPose(names[ii],newPose);
	  	markerServer->applyChanges();
	}

	//Clear all markers from interactive server
	//markerServer->clear();	

	for (auto obj : object_permanence) {
		//Skip objects that are not real
 		if (!obj.real) {
			continue;
		}

		//Display Info
		ROS_INFO_STREAM("LIDAR | " << obj.name << " " << obj.id << " at " << obj.position.x << "," << obj.position.y << "," << obj.position.z << " with " << obj.strikesPersist.size() << "(" << obj.strikesFrame.size() << ") points, size " << obj.scale.x << "," << obj.scale.y << "," << obj.scale.z << " maxHeight " << obj.maxHeightFromLidar << ", " << obj.current);

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
			
		//Create Interactive marker
		auto int_marker = CreateInteractiveMarker(to_string(obj.id));

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
		if (obj.locked) {
			m4.color.a = 0.6; m4.color.r = 0; m4.color.g = 0; m4.color.b = 1;
		} else {
			m4.color.a = 0.6; m4.color.r = 0; m4.color.g = 1; m4.color.b = 0;
		}
		int_marker.controls[0].markers.push_back( m4 );	

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
			int_marker.controls[0].markers.push_back( m3 );
		}

		//Create marker as bounding box
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
		if (obj.color.r == 0 && obj.color.g == 0 && obj.color.b == 0) {
			m2.color.a = 0.6; m2.color.r = 0.3; m2.color.g = 0.3; m2.color.b = 0.3;
		} else {
			m2.color.a = 0.6; m2.color.r = obj.color.r; m2.color.g = obj.color.g; m2.color.b = obj.color.b;
		}
		int_marker.controls[0].markers.push_back( m2 );
		
		//Turn obj into an interactive marker for rviz
		markerServer->insert( int_marker );
		menuHandler.apply( *markerServer, to_string(obj.id) );
	}	
	//Publish all data to ROS
	pubMarkers.publish(markers);
	pubCloudPersist.publish(objectCloudPersist);
	pubCloudFrame.publish(objectCloudFrame);
	pubCloudPCL.publish(pclCloud);
	markerServer->applyChanges();
	
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
	
	//Assume match not found and clear old info
        res.found = false;
	res.objects.clear();

	//Look for desired object by name in database
	if (req.name.size() > 2) {
		res.found = object_tracker.lookUpByName(req.name,res.objects);
	}

	//Process cmd portion after spliting request into 3 components
	auto split1 = req.cmd.find('=');
	auto split2 = req.cmd.find(',');
	if ( split1 != string::npos and split2 != string::npos ) {
		auto name = req.cmd.substr(0,split1);

		//Check if argument is x,y or r,g,b
		auto split3 = req.cmd.find(',', split2 + 1);
		if ( split3 != string::npos ){
			// It's r, g, b
			auto r = stod(req.cmd.substr(split1+1,split2-split1));
			auto g = stod(req.cmd.substr(split2+1,split3-split2));
			auto b = stod(req.cmd.substr(split3+1));

			std_msgs::ColorRGBA color;
			color.r = r;
			color.g = g;
			color.b = b;
			
			object_tracker.colorize(name, color);
		
		} else {

			auto x = stod(req.cmd.substr(split1+1,split2-split1));
			auto y = stod(req.cmd.substr(split2+1));
			cout << "LIDAR | Cmd is " << name << "," << x << "," << y << endl;
			//Set new position
			geometry_msgs::Pose newPose;
			newPose.position.x = x;
			newPose.position.y = y;
			newPose.position.z = 0;
			//Update database
			object_tracker.lock(name,newPose.position);
			//Update interactive markers
			markerServer->setPose(name,newPose);
			markerServer->applyChanges();
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Process mouse moves from Rviz for interactive markers
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void roiCallBack( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("LIDAR | " << feedback->marker_name << ": " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
	//Zero out z poisition from Rviz
	auto newPose = feedback->pose;
	newPose.position.z = 0;
	//Update database
	object_tracker.lock(feedback->marker_name,newPose.position);
	//Update interactive markers
	markerServer->setPose(feedback->marker_name,newPose);
	markerServer->applyChanges();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createROIS(string name)
{
	//Add ROI estimates
  	visualization_msgs::InteractiveMarker int_marker; 
  	visualization_msgs::InteractiveMarkerControl controlm,control;
  	int_marker.header.frame_id = "enu";
  	int_marker.scale = 1;
  	int_marker.name = name;
  	controlm.name = name;
  	controlm.always_visible = true;
  	controlm.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
	visualization_msgs::Marker m4;
	m4.header.stamp = ros::Time::now();
	m4.header.seq = 0;
	m4.header.frame_id = "enu";		
	m4.id = -1;
	m4.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m4.action = visualization_msgs::Marker::ADD;
	m4.pose.position.x = 0;
	m4.pose.position.y = 0;
	m4.pose.position.z = 2;
	m4.scale.z = 2;
	m4.text = name;
	m4.color.a = 1; m4.color.r = 1; m4.color.g = 1; m4.color.b = 1;
  	controlm.markers.push_back(m4);
	m4.type = visualization_msgs::Marker::SPHERE;
	m4.pose.position.x = 0;
	m4.pose.position.y = 0;
	m4.pose.position.z = 0;
	m4.scale.x = 2;
	m4.scale.y = 2;
	m4.scale.z = 2;
	controlm.markers.push_back(m4);
  	int_marker.controls.push_back(controlm);
	markerServer->insert( int_marker );	
	markerServer->setCallback(int_marker.name,&roiCallBack);
	markerServer->applyChanges();
	object_tracker.addROI(name);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Process Rviz menu option selections
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void markerCallBack( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("LIDAR MENU | Callback from " << feedback->menu_entry_id << "," << feedback->marker_name);
	const vector<string> menu{"ObjectType", "shooter", "dock", "scan_the_code", "totem", "start_gate", "buoy", "unknown"};
	
	for (auto &obj : object_tracker.saved_objects) {
		if (obj.id == stoi(feedback->marker_name)) { 
			if (feedback->menu_entry_id < 9) {
				obj.name = menu.at(feedback->menu_entry_id-1); 
				ROS_INFO_STREAM("LIDAR | Changing id of object " << obj.id << " to " << obj.name);
			} else if (feedback->menu_entry_id == 10 ) {
				obj.locked = true;
				ROS_INFO_STREAM("LIDAR | Changing status of object " << obj.id << " to locked");
			} else if (feedback->menu_entry_id == 11) {
				obj.locked = false;
				ROS_INFO_STREAM("LIDAR | Changing status of object " << obj.id << " to unlocked");
			} else if (feedback->menu_entry_id == 13) {
				obj.color.r = 1.f; obj.color.g = 0.f; obj.color.b = 0.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to red");
			} else if (feedback->menu_entry_id == 14) {
				obj.color.r = 1.f; obj.color.g = 1.f; obj.color.b = 1.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to white");
			} else if (feedback->menu_entry_id == 15) {
				obj.color.r = 0.01f; obj.color.g = 0.01f; obj.color.b = 0.01f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to black");
			}	else if (feedback->menu_entry_id == 16) {
				obj.color.r = 0.f; obj.color.g = 1.f; obj.color.b = 0.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to green");
			} else if (feedback->menu_entry_id == 17) {
				obj.color.r = 0.f; obj.color.g = 0.f; obj.color.b = 1.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to blue");
			} else if (feedback->menu_entry_id == 18) {
				obj.color.r = 1.f; obj.color.g = 1.f; obj.color.b = 0.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to yellow");
			}	else if (feedback->menu_entry_id == 19) {
				obj.color.r = 0.f; obj.color.g = 0.f; obj.color.b = 0.f;
				ROS_INFO_STREAM("LIDAR | Changing color of object " << obj.id << " to unknown");
			}											
			break; 
		}
	}	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Entry point to code
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
	pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/info_markers",10);

	//Publish PerceptionObjects
	pubObjects = nh.advertise<navigator_msgs::PerceptionObjectArray>("/database/objects",1);
	pubObjectsTimer = ros::Time::now();

	//Extra publishing for debugging...
	pubCloudPersist = nh.advertise<sensor_msgs::PointCloud>("/ira_persist",1);
	pubCloudFrame = nh.advertise<sensor_msgs::PointCloud>("/ira_frame",1);
	pubCloudPCL = nh.advertise<sensor_msgs::PointCloud>("/ira_pclcloud",1);

	//Service for object request
	ros::ServiceServer service = nh.advertiseService("/database/requests", objectRequest);

	//Check for bounds from parameter server on startup
	boundsClient = nh.serviceClient<navigator_msgs::Bounds>("/get_bounds");
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

	//Interactive Marker Menu Setup
	markerServer = new interactive_markers::InteractiveMarkerServer("/unclassified_markers","",false);
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menuHandler.insert( "Type" );
	menuEntry = menuHandler.insert( sub_menu_handle, "shooter", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "dock", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "scan_the_code", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "totem", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "start_gate", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "buoy", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "unknown", &markerCallBack );
	sub_menu_handle = menuHandler.insert( "Lock" );
	menuEntry = menuHandler.insert( sub_menu_handle, "True", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "False", &markerCallBack );
	sub_menu_handle = menuHandler.insert( "Color" );
	menuEntry = menuHandler.insert( sub_menu_handle, "Red", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "White", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "Black", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "Green", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "Blue", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "Yellow", &markerCallBack );
	menuEntry = menuHandler.insert( sub_menu_handle, "Unknown", &markerCallBack );

	//Create ROIs
	createROIS("BuoyField");
	createROIS("CoralSurvey");
	createROIS("FindBreak");
	createROIS("AcousticPinger");
	createROIS("Shooter");
	createROIS("Scan_The_Code");
	createROIS("Gate_1");
	createROIS("Gate_2");
	createROIS("Gate_3");
	createROIS("Dock");

	//Give control to ROS
	ros::spin();

	return 0;
}
