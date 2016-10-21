#include <navigator_vision_lib/pcd_colorizer.hpp>

using namespace std;
using namespace cv;

namespace nav{

PcdColorizer::PcdColorizer(ros::NodeHandle nh, string input_pcd_topic, string output_pcd_topic, string rgb_cam_topic, string rgb_cam_frame)
: PcdSubPubAlgorithm(nh, input_pcd_topic, output_pcd_topic)
{
  this->rgb_cam_topic = rgb_cam_topic;
  this->rgb_cam_frame = rgb_cam_frame;

  _cloud_sub = nh.subscribe(input_pcd_topic, 10, &PcdColorizer::cloud_cb, this);
  _cloud_pub = nh.advertise<PCD>(output_pcd_topic, 10, false);
  rgb_cam_sub = img_transport.subscribeCamera(rgb_cam_topic, 10, 
    [this](const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
    {
      this->latest_frame_img_msg = image_msg_ptr;
      this->latest_frame_info_msg = info_msg_ptr;
      if(!_intrinsics_set)
      {
        auto K = info_msg_ptr->K;
        cam_intrinsics << K[0], K[1], K[2],
                          K[3], K[4], K[5],
                          K[6], K[7], K[8];
        _intrinsics_set = true;
      }
    }
    );
}


void PcdColorizer::cloud_cb(const PCD &cloud_msg)
{
  _input_pcd = cloud_msg;
  _transform_to_cam();
  _color_pcd();
  _cloud_pub.publish(_output_pcd);
  seq++;
}


void PcdColorizer::_transform_to_cam()
{
  tf::StampedTransform vel_to_cam_tf;
  ros::Time vel_cloud_stamp = _input_pcd.header.stamp;
  if(ros::ok()){
    try{
      tf_listener.lookupTransform(rgb_cam_frame, _input_pcd.header.frame_id, vel_cloud_stamp, vel_to_cam_tf);
    }
    catch(std::exception &e){
      cout << "Unable to look up tf between pcd and rgb camera. Error: " << e.what() << endl;
    }
  }
  pcl_ros::transformPointCloud(rgb_cam_frame, vel_to_cam_tf, _input_pcd, transformed_pcd);
  // alternative: pcl_ros::transformPointCloud(rgb_cam_frame, _input_pcd, pcd_cam, tf_listener);
}


void PcdColorizer::_color_pcd()
{
  // Confgure output_pcd header
  _output_pcd = PCD();
  _output_pcd.header.stamp = _input_pcd.header.stamp;
  _output_pcd.header.seq = seq;
  _output_pcd.header.frame_id = rgb_cam_frame;
  _output_pcd.height = 1;
  _output_pcd.width = _input_pcd.width; 
  _output_pcd.is_bigendian = false;
  _output_pcd.is_dense = false; 

  // Create classes to iterate over and modify clouds
  sensor_msgs::PointCloud2Iterator<float> iter_x_input{transformed_pcd, "x"};
  sensor_msgs::PointCloud2Modifier output_pcd_modifier{_output_pcd};
  output_pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  sensor_msgs::PointCloud2Iterator<float> iter_x{_output_pcd, "x"};
  sensor_msgs::PointCloud2Iterator<float> iter_y{_output_pcd, "y"};
  sensor_msgs::PointCloud2Iterator<float> iter_z{_output_pcd, "z"};
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(_output_pcd, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(_output_pcd, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(_output_pcd, "r");

  // Get latest frame
  Mat latest_frame_mat;
  if(latest_frame_img_msg == nullptr)
    return;
  cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(latest_frame_img_msg, sensor_msgs::image_encodings::BGR8);
  latest_frame_mat = input_bridge->image;

  // lambda for checking if point is visible from camera
  auto in_view = [=](Eigen::Vector3f v)
      {return v[0] >= 0 && v[0] < latest_frame_mat.cols && v[1] >= 0 && v[1] < latest_frame_mat.rows;};

  // Do work on point cloud
  while(iter_x_input != iter_x_input.end())
  {
    // Get image coordinates of 3D point
    Eigen::Vector3f xyz;
    xyz << iter_x_input[0], iter_x_input[1], iter_x_input[2];
    Eigen::Vector3f hom_img_coordinates;
    hom_img_coordinates = cam_intrinsics * xyz; // Project point in cam frame into image plane
    hom_img_coordinates = hom_img_coordinates / hom_img_coordinates[2];  // normalize homogenous vector

    // Fill in RGB info
    Vec3b rgb; 
    if(in_view(hom_img_coordinates))
    {
      rgb = latest_frame_mat.at<Vec3b>(hom_img_coordinates[1], hom_img_coordinates[0]);  // at takes in (y, x)
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

    // Increment cloud msg field iterators
    ++iter_x_input;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
}

} // namespace nav