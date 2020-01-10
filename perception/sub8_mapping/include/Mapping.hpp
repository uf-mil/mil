
#include <geometry_msgs/Point.h>
#include <mil_blueview_driver/BlueViewPing.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdexcept>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/circular_buffer.hpp>

#include <sub8_msgs/Bounds.h>

#include <waypoint_validity.hpp>

//#include <sub8_gazebo/simulated_sonar_ping.h>
#include <mil_msgs/ObjectDBQuery.h>
#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>
#include <mil_msgs/RangeStamped.h>
#include <std_srvs/Trigger.h>
#include <ros_alarms/listener.hpp>

#include <string>
#include <boost/filesystem.hpp> 

extern struct map_param
{
	bool map_pc;								// Show the point cloud 
	bool use_sonar;							// Weather or not to use sonar's world data
	bool map_objects;
	bool show_markers;						// Display latest DVL depth marker in Rviz
	float max_depth;  					// maximum depth in meters to map
	int pcl_buffer;

} params;


class Mapping

{
public:
	Mapping();
	void dvl_callback(const mil_msgs::RangeStampedConstPtr &dvl);
	//void sonar_callback(const sub8_gazebo::simulated_sonar_pingPtr &ping_msg);
	//void object_callback(const mil_msgs::PerceptionObjectArray &objects);
	void process_image(std::string frame_id, int id, cv::Mat& pic, tf::StampedTransform transform_);

private:
	ros::NodeHandle nh_;
	tf::TransformListener listener_;
	tf::StampedTransform transform_;
	ros_alarms::AlarmListener<> kill_listener_;
	bool was_killed_;

	ros::Subscriber sub_dvl_;
	//ros::Subscriber sub_imaging_sonar_;
	//ros::Subscriber sub_ogrid_objs_;

	// Publishers
	ros::Publisher pub_surface_;
	ros::Publisher pub_floor_;


	double dvl_depth_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud_;
	cv::Mat surface_image_;
	cv::Mat floor_image_;

	std::string floor_img_path_;
	std::string surface_img_path_;
	bool use_image_;
	bool surface_marker_;
	bool use_down_cam_;



};
