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
#include <uf_common/PoseTwistStamped.h>
#include <uf_common/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "AStar.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500.3;
const double ROI_SIZE_METERS = 201.3;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 50; //20
const int MAX_HITS_IN_CELL = 500; //500
const double MAXIMUM_Z_HEIGHT = 8;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS,ROI_SIZE_METERS,VOXEL_SIZE_METERS);
AStar astar(ROI_SIZE_METERS/VOXEL_SIZE_METERS);
nav_msgs::OccupancyGrid rosGrid;
ros::Publisher pubGrid,pubMarkers,pubBuoys,pubTrajectory,pubWaypoint;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point waypoint_enu, waypoint_ogrid;
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
uf_common::PoseTwistStamped carrot_enu;
actionlib::SimpleActionServer<uf_common::MoveToAction> *actionServerPtr;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_velodyne(const sensor_msgs::PointCloud2ConstPtr &pcloud) 
{
	ROS_INFO("**********************************************************");
	ROS_INFO("LIDAR | cb_velodyne...");	

	//Measure elapsed time for function
	ros::Time timer = ros::Time::now();

	//Get new action
	if (actionServerPtr->isNewGoalAvailable()) {
      	boost::shared_ptr<const uf_common::MoveToGoal> goal = actionServerPtr->acceptNewGoal();
      	//goal->posetwist.pose, goal->posetwist.twist), goal->speed,!goal->uncoordinated
    }

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
    geometry_msgs::Vector3  lidarpos =  T_enu_velodyne_ros.transform.translation;
    geometry_msgs::Quaternion quat = T_enu_velodyne_ros.transform.rotation;
	T_enu_velodyne.translate(Eigen::Vector3d(lidarpos.x,lidarpos.y,lidarpos.z));
	T_enu_velodyne.rotate(Eigen::Quaterniond(quat.w,quat.x,quat.y,quat.z));
	ROS_INFO_STREAM("LIDAR | Velodyne enu: " << lidarpos.x << "," << lidarpos.y << "," << lidarpos.z);

	//Update occupancy grid
	ogrid.setLidarPosition(lidarpos);
	ogrid.updatePointsAsCloud(pcloud,T_enu_velodyne,MAX_HITS_IN_CELL);
	ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY,MAXIMUM_Z_HEIGHT);

	//Inflate ogrid before detecting objects and calling AStar
	ogrid.inflateBinary(6);

	//Detect objects
	std::vector<objectMessage> objects;
	std::vector< std::vector<int> > cc = ConnectedComponents(ogrid,objects);



	//Fake waypoint - this needs to be replaced! //Eigen::Vector3d v = T_enu_velodyne*Eigen::Vector3d(30,0,0);
	waypoint_enu.x = 25;
	waypoint_enu.y = -25;
	waypoint_enu.z = 5;
			
	//Convert waypoint from enu frame to ROI around lidar
	waypoint_ogrid.x = (int)( (waypoint_enu.x-lidarpos.x)/VOXEL_SIZE_METERS + ogrid.ROI_SIZE/2 );
	waypoint_ogrid.y = (int)( (waypoint_enu.y-lidarpos.y)/VOXEL_SIZE_METERS + ogrid.ROI_SIZE/2 );
	waypoint_ogrid.z = waypoint_enu.z;

	//Force waypoint to fit on ROI
	if (waypoint_ogrid.x >= ogrid.ROI_SIZE) { waypoint_ogrid.x = ogrid.ROI_SIZE-1; }
	if (waypoint_ogrid.x < 0) { waypoint_ogrid.x = 0; }	
	if (waypoint_ogrid.y >= ogrid.ROI_SIZE) { waypoint_ogrid.y = ogrid.ROI_SIZE-1; }	
	if (waypoint_ogrid.y < 0) { waypoint_ogrid.y = 0; }	

	//Inflate waypoint on grid for easier visuals
	for (int ii = -2; ii <= 2; ++ii) {
		for (int jj = -2; jj <= 2; ++jj) {
			ogrid.ogridMap[ (waypoint_ogrid.y+ii)*ogrid.ROI_SIZE + waypoint_ogrid.x+jj] = 25;
		}
	}

	//Setup Astar
	astar.setMap(ogrid.ogridBinary);
	astar.setFinish(waypoint_ogrid.x, waypoint_ogrid.y);	

	//Mark starting square on map
	ogrid.ogridMap[ (astar.startNode.y)*ogrid.ROI_SIZE + astar.startNode.x] = 25;

	//Run Astar
	auto solution = astar.run();
	
	//Set the carrot as the boat's current position and orientation - this is the backup for the boat not to move if Astar fails
	carrot_enu.posetwist.pose.position = boatPose_enu.position; //waypoint_enu;
	carrot_enu.posetwist.pose.orientation = boatPose_enu.orientation;
	carrot_enu.posetwist.twist.linear.x = 0;
	carrot_enu.posetwist.twist.linear.y = 0;
	carrot_enu.posetwist.twist.linear.z = 0;
	carrot_enu.posetwist.twist.angular.x = 0;
	carrot_enu.posetwist.twist.angular.y = 0;
	carrot_enu.posetwist.twist.angular.z = 0;

	//If the Astar path still has steps to go, pick a spot a few iterations ahead of the boat
	Eigen::Vector3d desHeading(0,0,0);
	double desBoatRotAngle = 0;
	
	//If we are close to the goal, make that the waypoing
	if (solution.size() <= 10) {
		carrot_enu.posetwist.pose.position = waypoint_enu;
		//carrot_enu.posetwist.pose.orientation = WAYPOINT ORIENTATION 
	}

	//Determine the boats current rotation in the plane of the water
	double boatRotAngle = atan2(Eigen::Vector3d(boatPose_enu.orientation.y,boatPose_enu.orientation.x,boatPose_enu.orientation.z).norm(),boatPose_enu.orientation.w)*2*180/3.14159;
	ROS_INFO_STREAM("LIDAR | Boat Rotation angle: " << boatRotAngle);

	int index = 0;
	for (auto square : solution) {
		//Place path on map
		ogrid.ogridMap[square.second*ogrid.ROI_SIZE+square.first] = 0;

		//At some arbitrary distance away, make this the desired waypoint
		if (index == 20) {
			//Determine heading and corresponding angle
			desHeading(0) = square.first-ogrid.ROI_SIZE/2; 
			desHeading(1) = square.second-ogrid.ROI_SIZE/2;
			desHeading(2) = 0; 
			desHeading.normalize();
			desBoatRotAngle = atan2( desHeading(1), desHeading(0) ) ;
			ROS_INFO_STREAM("Desired Heading: " << desHeading(0) << "," << desHeading(1) << " -> " << desBoatRotAngle*180/3.14159);

			//Update carrot in the enu
			//We only let the boat move if we are pointing close to the right direction
			if (fabs(desBoatRotAngle - boatRotAngle) < 15) {
				carrot_enu.posetwist.pose.position.x = (square.first-ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.x - ogrid.VOXEL_SIZE_METERS;
				carrot_enu.posetwist.pose.position.y = (square.second-ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.y - ogrid.VOXEL_SIZE_METERS;
				carrot_enu.posetwist.twist.linear.x = desHeading(0)*1;
				carrot_enu.posetwist.twist.linear.y = desHeading(1)*1;
				carrot_enu.posetwist.twist.linear.z = 0;
			}
			carrot_enu.posetwist.pose.orientation.w = cos(desBoatRotAngle/2);
			carrot_enu.posetwist.pose.orientation.x = 0;
			carrot_enu.posetwist.pose.orientation.y = 0;
			carrot_enu.posetwist.pose.orientation.z = 1*sin(desBoatRotAngle/2);


			//Make the carrot look bigger than it is
			int ii = 0, jj = 0;
			for (int ii = -2; ii <= 2; ++ii) {
				for (int jj = -2; jj <= 2; ++jj) {
					ogrid.ogridMap[ (square.second+ii)*ogrid.ROI_SIZE+square.first+jj] = 75;
				}
			}
		}
		++index;
	}
	pubTrajectory.publish(carrot_enu);
	ROS_INFO_STREAM("LIDAR | Carrot enu: " << carrot_enu.posetwist.pose.position.x << "," << carrot_enu.posetwist.pose.position.y << "," << carrot_enu.posetwist.pose.position.z);
	ROS_INFO_STREAM("LIDAR | Boat enu: " << boatPose_enu.position.x << "," << boatPose_enu.position.y << "," << boatPose_enu.position.z);
	ROS_INFO_STREAM("LIDAR | Carrot orientation: " << carrot_enu.posetwist.pose.orientation.w << "," << carrot_enu.posetwist.pose.orientation.x << "," << carrot_enu.posetwist.pose.orientation.y << "," << carrot_enu.posetwist.pose.orientation.z);
	ROS_INFO_STREAM("LIDAR | Boat orientation: " << boatPose_enu.orientation.w << "," << boatPose_enu.orientation.x << "," << boatPose_enu.orientation.y << "," << boatPose_enu.orientation.z);




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

	//Course Outline - change to real values or pull from service/topic
	m.id = 1;
	m.type = visualization_msgs::Marker::LINE_STRIP;
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = 0.5;
	p.x = -25; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
	p.x = -25; p.y = 0; p.z = lidarpos.z; m.points.push_back(p);
	p.x = 35; p.y = 0; p.z = lidarpos.z; m.points.push_back(p);
	p.x = 35; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
	p.x = -25; p.y = 50; p.z = lidarpos.z; m.points.push_back(p);
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
		//Verify object is in the enu frame - this should come from LLA transforms later
		//if (obj.position.x < -35 || obj.position.x > 65 || obj.position.y < -35 || obj.position.y > 50 ) { continue; }
		if (obj.position.x < -25 || obj.position.x > 35 || obj.position.y < 0 || obj.position.y > 50 ) { continue; }
		ROS_INFO_STREAM("LIDAR | Adding buoy " << id << " at " << obj.position.x << "," << obj.position.y << "," << obj.position.z << " with " << obj.beams.size() << " lidar points ");
		//for (auto jj : obj.beams) {
		//1	std::cout << jj.x << "," << jj.y << "," << jj.z << "," << jj.i << std::endl;
		//}

		//Buoys
		buoy.header.stamp = ros::Time::now();
		buoy.id = id;
		buoy.confidence = 0;
		buoy.position = obj.position;
		buoy.height = obj.scale.z; 
		buoy.width = obj.scale.x; //x or y for width?
		buoy.points = obj.beams;
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
	pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("ogrid_batcave",10);
	pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("markers_batcave",10);
	pubBuoys = nh.advertise<navigator_msgs::BuoyArray>("buoys_batcave",10);

	//Publish waypoints to controller
	pubTrajectory = nh.advertise<uf_common::PoseTwistStamped>("trajectory", 1);
    //pubWaypoint = nh.advertise<PoseStamped>("waypoint", 1); //Do we need this?

	//Setup action server
	actionlib::SimpleActionServer<uf_common::MoveToAction> actionServer(nh, "moveto", false);
	actionServerPtr = &actionServer;

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
