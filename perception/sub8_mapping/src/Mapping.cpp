#include "Mapping.hpp"

//map_param params;

Mapping::Mapping()
	: nh_(ros::this_node::getName())
  , kill_listener_(nh_, "kill")
  , was_killed_(true)
  , pointCloud_(new pcl::PointCloud<pcl::PointXYZI>())
 {
 
	// Pubishers
 	pub_surface_     = nh_.advertise<visualization_msgs::Marker>("surface", 1);
 	pub_floor_			 = nh_.advertise<visualization_msgs::Marker>("bottom", 1);

 	/* // Get these working
 	nh_.param<bool>("map_pc" params.map_pc, false);
 	nh_.param<bool>("show_markers" params.show_markers, false);
 	nh_.param<bool>("use_sonar" params.use_sonar, false);
 	nh_.param<float>("max_depth" params.max_depth, 37);
 	nh_.param<int>("pcl_buffer" params.pcl_buffer, 5000);
 	nh_.param<bool>("map_objects" params.map_objects, false);
 	nh_.param<std::string>("sonar_topic" params.sonar_topic, "/blueview_driver/ranges");
	*/

 	//sub_imaging_sonar_ = nh_.subscribe("/blueview/sonar_link", 1, &Mapping::sonar_callback, this);
 	sub_dvl_					 = nh_.subscribe("/dvl/range", 1, &Mapping::dvl_callback, this);
 	/*
 	try
 	{
 		sub_ogrid_objs_ = nh_.subscribe("/objects", 1, &Mapping::object_callback, this);
 	}catch ( const tf2::LookupException& ex )
	{
    ROS_ERROR_STREAM("Mapper could not subscribe to '/objects'");
	}
*/
  //TODO make these arguments in launch file
	use_image_ = true;
  use_down_cam_ = false;

	floor_img_path_ = "../mil_ws/src/SubjuGator/perception/sub8_mapping/include/seaFloor.png";
  if (!use_down_cam_)
  {
    try
    {
      floor_image_ = cv::imread(floor_img_path_, cv::IMREAD_COLOR);
    }
    catch( cv::Exception& ex )
    {
      const char* err_msg = ex.what();
      ROS_ERROR_STREAM( err_msg );
      use_image_ = false;
    }

    if (floor_image_.empty() )
    {
      boost::filesystem::path full_path(boost::filesystem::current_path());
      ROS_ERROR_STREAM("Empty ground image. Defaulting to some BS /n" << full_path );
      use_image_ = false;
    }
  }
	
  surface_marker_ = true;
  surface_img_path_ = "../mil_ws/src/SubjuGator/perception/sub8_mapping/include/surface.png";
  if(surface_marker_)
  {
    try
    {
      surface_image_ = cv::imread(surface_img_path_, cv::IMREAD_COLOR);
    }
    catch( cv::Exception& ex )
    {
      const char* err_msg = ex.what();
      ROS_ERROR_STREAM( err_msg );
      use_image_ = false;
    }

    if (surface_image_.empty() )
    {
      boost::filesystem::path full_path(boost::filesystem::current_path());
      ROS_ERROR_STREAM("Empty surface image. Defaulting to some BS /n" << full_path );
      use_image_ = false;
    }
  }
  
 	dvl_depth_ = 0;

	kill_listener_.waitForConnection(ros::Duration(2)); 
	if (kill_listener_.getNumConnections() < 1)
		throw std::runtime_error("The kill listener is not connected to alarm server!");
	kill_listener_.start();
 }


void Mapping::dvl_callback(const mil_msgs::RangeStampedConstPtr &dvl)
{
	try
	{
		listener_.lookupTransform("/map", "/dvl", ros::Time(0), transform_);
	}catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM("Did not get TF for DVL");
    return;
  }

   if (kill_listener_.isRaised())
    was_killed_ = true;
  else if (was_killed_)
  {
    was_killed_ = false;
  }

  int id = 0;
  dvl_depth_ = dvl->range;

  if(use_image_)
  {

	process_image("map", id++, floor_image_, transform_);
  if(surface_marker_)
    process_image("map", id++, surface_image_, transform_);
	return;
  }

	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = "map";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.pose.position.x = transform_.getOrigin().x();
	marker.pose.position.y = transform_.getOrigin().y();
	marker.pose.position.z = (0 - dvl_depth_);
	marker.scale.x = .75;
	marker.scale.y = .75;
	marker.scale.z = .01;
	marker.color.a = 1.0;
	marker.color.b = 0.0;
	marker.color.g = 1.0;
	marker.color.r = 0.0;
	pub_floor_.publish(marker);
}

void Mapping::process_image(std::string frame_id, int id, cv::Mat& pic, tf::StampedTransform transform_)
{
	
	visualization_msgs::Marker image;
	image.header.frame_id = "map";
	image.header.stamp = ros::Time::now();
	//image.ns = "image";
	image.id = id;
	image.action = visualization_msgs::Marker::ADD;
	image.type = visualization_msgs::Marker::TRIANGLE_LIST;
	image.scale.x = 1;
	image.scale.y = 1;
	image.scale.z = 1;

	//TODO: add an launch arg for scaling
	float scale_size = 1.0;
  float scale_rows = scale_size / pic.rows;
  float scale_cols = scale_size / pic.cols;
  float tf_x = transform_.getOrigin().x() - 0.5;
	float tf_y = transform_.getOrigin().y() - 0.5;
  geometry_msgs::Point p;
  std_msgs::ColorRGBA crgb;

  for(int r= pic.rows-1; r > 0; --r)
  {
  	for( int c = pic.cols; c > 0; --c)
  	{
  		cv::Vec3b intensity = pic.at<cv::Vec3b>(r, c);

      if(!id)
  		  crgb.a = 1.0;
      else
        crgb.a = 0.5;

  		crgb.r = intensity.val[2] / 255.0;
  		crgb.g = intensity.val[1] / 255.0;
      crgb.b = intensity.val[0] / 255.0;

      if(!id) p.z = 0 - dvl_depth_;
      else p.z = 0;

      p.x =  tf_x + r * scale_rows;
      p.y = tf_y + c * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);
      p.x =  tf_x + (r + 1) * scale_rows;
      p.y = tf_y + c * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);
      p.x =  tf_x + r * scale_rows;
      p.y = tf_y + (c + 1) * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);
      p.x =  tf_x + (r + 1) * scale_rows;
      p.y = tf_y + c * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);
      p.x =  tf_x + (r + 1) * scale_rows;
      p.y = tf_y + (c + 1) * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);
      p.x =  tf_x + r * scale_rows;
      p.y = tf_y + (c + 1) * scale_cols;
      image.points.push_back(p);
      image.colors.push_back(crgb);

  	}
  }
  if(!id)
   pub_floor_.publish(image);
  else
    pub_surface_.publish(image);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub8_mapping");
  Mapping map;
  ros::spin();
}

/* // Future expansion
void Mapping::sonar_callback(const sub8_gazebo::simulated_sonar_pingPtr &ping_msg)
{

  try  
  {
    listener_.lookupTransform("/map", "/blueview/sonar_link", ros::Time(0), transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG_STREAM("Did not get TF for imaging sonar");
    return;
  }
}


void Mapping::object_callback(const mil_msgs::PerceptionObjectArray &objects)
{

}
*/