#include <navigator_vision_lib/point_cloud_algorithms.hpp>

using namespace std;
using namespace cv;

PcdColorizer::PcdColorizer(string input_pcd_topic, string output_pcd_topic, string rgb_cam_topic, string rgb_cam_frame)
{
  this->input_pcd_topic = input_pcd_topic;
  this->output_pcd_topic = output_pcd_topic;
  this->rgb_cam_topic = rgb_cam_topic;
  this->rgb_cam_frame = rgb_cam_frame;

  cloud_sub = nh.subscribe(input_pcd_topic, 10, &PcdColorizer::cloud_cb, this);
  cloud_pub = nh.advertise<PCD>(output_pcd_topic, 10, false);
  img_transport.subscribeCamera(rgb_cam_topic, 10, 
    [this](const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
    {latest_frame_img_msg = image_msg_ptr; latest_frame_info_msg = info_msg_ptr;});
}

void PcdColorizer::cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg){
  input_pcd = cloud_msg;
  _transform_to_cam();
  _color_pcd();
  cloud_pub.publish(output_pcd);
}

void PcdColorizer::_transform_to_cam(){
  tf::StampedTransform vel_to_cam_tf;
  ros::Time vel_cloud_stamp = input_pcd.header.stamp;
  if(ros::ok()){
    try{
      tf_listener.lookupTransform(input_pcd.header.frame_id, rgb_cam_frame, vel_cloud_stamp, vel_to_cam_tf);
    }
    catch(std::exception &e){
      cout << "Unable to look up tf between pcd and rgb camera. Error: " << e.what() << endl;
    }
  }
  pcl_ros::transformPointCloud(rgb_cam_frame, vel_to_cam_tf, input_pcd, transformed_pcd);
  // pcl_ros::transformPointCloud(rgb_cam_frame, input_pcd, pcd_cam, tf_listener);

}

void PcdColorizer::_color_pcd(){
  // Create classes to iterate over and modify clouds
  sensor_msgs::PointCloud2Iterator<float> iter_x_input(transformed_pcd, "x");
  sensor_msgs::PointCloud2Modifier output_pcd_modifier{output_pcd};
  output_pcd_modifier.clear();
  output_pcd_modifier.setPointCloud2Fields(2, "xyz", "rgb");
  sensor_msgs::PointCloud2Iterator<float> iter_x_output(output_pcd, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb_output(output_pcd, "rgb");

  // Latest frame
  Mat latest_frame_mat;

  while(iter_x_input != iter_x_input.end()){
    Eigen::Vector3f xyz;
    xyz << iter_x_input[0], iter_x_input[1], iter_x_input[2];
    Eigen::Vector3f hom_img_coordinates;
    hom_img_coordinates = cam_intrinsics * xyz; // Project point in cam frame into image plane
    hom_img_coordinates / hom_img_coordinates[2];  // normalize homogenous vector
    cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(latest_frame_img_msg, sensor_msgs::image_encodings::BGR8);
    Vec3b rgb = latest_frame_mat.at<Vec3b>(hom_img_coordinates[0], hom_img_coordinates[1]);

    // Set the values of the fields of thecolored output cloud
    iter_x_output[0] = xyz[0];
    iter_x_output[1] = xyz[1];
    iter_x_output[2] = xyz[2];
    iter_rgb_output[0] = rgb[0];
    iter_rgb_output[1] = rgb[1];
    iter_rgb_output[2] = rgb[2];


    ++iter_x_input;
    ++iter_x_output;
    ++iter_rgb_output;


  }

}