#include <navigator_vision_lib/point_cloud_algorithms.hpp>

using namespace std;
using namespace cv;

namespace nav{

Mat g_color_sequence;

PcdColorizer::PcdColorizer(ros::NodeHandle nh, string input_pcd_topic, string output_pcd_topic, string rgb_cam_topic, string rgb_cam_frame)
{
  this->nh = nh;
  this->input_pcd_topic = input_pcd_topic;
  this->output_pcd_topic = output_pcd_topic;
  this->rgb_cam_topic = rgb_cam_topic;
  this->rgb_cam_frame = rgb_cam_frame;

  cloud_sub = nh.subscribe(input_pcd_topic, 10, &PcdColorizer::cloud_cb, this);
  cloud_pub = nh.advertise<PCD>(output_pcd_topic, 10, false);
  rgb_cam_sub = img_transport.subscribeCamera(rgb_cam_topic, 10, 
    [this](const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
    {
      this->latest_frame_img_msg = image_msg_ptr;
      this->latest_frame_info_msg = info_msg_ptr;
      if(!_intrinsics_set){
        auto K = info_msg_ptr->K;
        cam_intrinsics << K[0], K[1], K[2],
                          K[3], K[4], K[5],
                          K[6], K[7], K[8];
        _intrinsics_set = true;
      }
    }
    );
}

void PcdColorizer::cloud_cb(const PCD &cloud_msg){
  input_pcd = cloud_msg;
  _transform_to_cam();
  _color_pcd();
  cloud_pub.publish(output_pcd);
  seq++;
}

void PcdColorizer::_transform_to_cam(){
  tf::StampedTransform vel_to_cam_tf;
  ros::Time vel_cloud_stamp = input_pcd.header.stamp;
  if(ros::ok()){
    try{
      // tf_listener.lookupTransform(input_pcd.header.frame_id, rgb_cam_frame, vel_cloud_stamp, vel_to_cam_tf);
      tf_listener.lookupTransform(rgb_cam_frame, input_pcd.header.frame_id, vel_cloud_stamp, vel_to_cam_tf);
    }
    catch(std::exception &e){
      cout << "Unable to look up tf between pcd and rgb camera. Error: " << e.what() << endl;
    }
  }
  pcl_ros::transformPointCloud(rgb_cam_frame, vel_to_cam_tf, input_pcd, transformed_pcd);
  // pcl_ros::transformPointCloud(rgb_cam_frame, input_pcd, pcd_cam, tf_listener);

}

void PcdColorizer::_color_pcd(){
  // confgure output_pcd header
  output_pcd = PCD();
  output_pcd.header.stamp = input_pcd.header.stamp;
  output_pcd.header.seq = seq;
  output_pcd.header.frame_id = rgb_cam_frame;
  // cout << "OUT CLOUD frame:" << rgb_cam_frame << endl;
  output_pcd.height = 1;
  output_pcd.width = input_pcd.width; 
  output_pcd.is_bigendian = false;
  output_pcd.is_dense = false; 

  // Create classes to iterate over and modify clouds
  sensor_msgs::PointCloud2Iterator<float> iter_x_input{transformed_pcd, "x"};
  sensor_msgs::PointCloud2Modifier output_pcd_modifier{output_pcd};
  output_pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  sensor_msgs::PointCloud2Iterator<float> iter_x{output_pcd, "x"};
  sensor_msgs::PointCloud2Iterator<float> iter_y{output_pcd, "y"};
  sensor_msgs::PointCloud2Iterator<float> iter_z{output_pcd, "z"};
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output_pcd, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output_pcd, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output_pcd, "r");
  // cout << "iter_x_output: " << iter_x_output.data_ << endl;
  // cout << "iter_rgb_output: " << iter_rgb_output.data_ << endl;

  // Latest frame
  Mat latest_frame_mat;
  cout << "latest_frame_ptr:" << latest_frame_img_msg << endl;
  if(latest_frame_img_msg == nullptr)
    return;
  // cout << "Getting frame:" << latest_frame_img_msg << endl;
  cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(latest_frame_img_msg, sensor_msgs::image_encodings::BGR8);
  // cout << "extracting frame" << endl;
  latest_frame_mat = input_bridge->image;
  // cout << "There is no god!" << endl;
  cout << "K: " << cam_intrinsics << endl;



  // lambda for checking if point is visible from camera
  auto in_view = [=](Eigen::Vector3f v)
      {return v[0] >= 0 && v[0] < latest_frame_mat.cols && v[1] >= 0 && v[1] < latest_frame_mat.rows;};

  while(iter_x_input != iter_x_input.end())
  {
    Eigen::Vector3f xyz;
    xyz << iter_x_input[0], iter_x_input[1], iter_x_input[2];
    Eigen::Vector3f hom_img_coordinates;
    hom_img_coordinates = cam_intrinsics * xyz; // Project point in cam frame into image plane
    hom_img_coordinates = hom_img_coordinates / hom_img_coordinates[2];  // normalize homogenous vector
    Vec3b rgb;
    if(in_view(hom_img_coordinates)){
      // cout << "hom_coords: " << hom_img_coordinates << " cols: " << latest_frame_mat.cols << " rows: " 
        // << latest_frame_mat.rows << endl;
      rgb = latest_frame_mat.at<Vec3b>(hom_img_coordinates[0], hom_img_coordinates[1]);
    }
    else
    {
      rgb[0] = 122;
      rgb[1] = 122;
      rgb[2] = 122;
    }

    // Set the values of the fields of the colored output cloud
    *iter_x = xyz[0];
    *iter_y = xyz[1];
    *iter_z = xyz[2];
    *iter_b = rgb[0];
    *iter_g = rgb[1];
    *iter_r = rgb[2];

    ++iter_x_input;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
}

} //namespace nav