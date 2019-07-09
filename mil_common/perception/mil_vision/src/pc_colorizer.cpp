#include <ros/ros.h>
#include <deque>

#include <sensor_msgs/PointCloud2.h>

#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using ROSCameraStream_Vec3 = mil_vision::ROSCameraStream<cv::Vec3b>;
//! Point Cloud Colorizer class.
/*! Subscribes to cameras and velodyne topic to publish a RGB pointcloud.
  Inputs: rosparam '/pc_colorizer/cameras', and topic '/velodyne_points'
  Outputs: topic '/velodyne_points/colored'
*/
class Colorizer
{
  ros::NodeHandle nh_;
  //! Deque to store camera streams for each camera
  std::deque<std::unique_ptr<ROSCameraStream_Vec3>> cameras_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher rgb_cloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  //! Initialize tf buffer and resolve camera feeds
  Colorizer() : nh_(), tf_listener_(tf_buffer_)
  {
    std::vector<std::string> camera_names = { "/camera/front/right/image_rect_color",
                                              "/camera/front/left/image_rect_color", "/camera/right/image_rect_color" };
    nh_.param<std::vector<std::string>>("cameras", camera_names, camera_names);

    // what topic to subscribe to
    std::string pointcloud_subscriber = "/velodyne_points";
    nh_.param<std::string>("pointcloud_subscriber", pointcloud_subscriber, pointcloud_subscriber);

    // what should the publisher topic name be
    std::string pointcloud_advertiser = "/velodyne_points/colored";
    nh_.param<std::string>("pointcloud_advertiser", pointcloud_advertiser, pointcloud_advertiser);

    // pushback camera streams that are avaliable
    for (auto &i : camera_names)
    {
      std::unique_ptr<ROSCameraStream_Vec3> cam(new ROSCameraStream_Vec3(nh_, 1));
      cam->init(i);
      if (cam->ok())
        cameras_.push_back(std::move(cam));
      else
        ROS_ERROR_STREAM("PC_COLORIZER: failed to subscribe to " << i);
    }
    point_cloud_sub_ =
        nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(pointcloud_subscriber, 1, &Colorizer::cb_velodyne, this);
    rgb_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(pointcloud_advertiser, 1);
  }

  void cb_velodyne(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &point_cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*point_cloud));

    // Remember transformation from pointcloud to camera frame
    std::vector<Eigen::Affine3d> tf_cameras(cameras_.size());
    auto tf_cameras_it = tf_cameras.begin();
    for (auto &&cam : cameras_)
    {
      if (!tf_buffer_.canTransform((*cam)[0]->getCameraModelPtr()->tfFrame(), point_cloud->header.frame_id,
                                   ros::Time(0)))
      {
        ROS_ERROR_STREAM("PC_COLORIZER: failed to find a trasformation for camera");
        return;
      }

      // Get TF and convert to eigen type
      geometry_msgs::TransformStamped transform_cam_velodyne_geom = tf_buffer_.lookupTransform(
          (*cam)[0]->getCameraModelPtr()->tfFrame(), point_cloud->header.frame_id, ros::Time(0));
      Eigen::Affine3d transform_cam_velodyne(Eigen::Affine3d::Identity());

      geometry_msgs::Vector3 position = transform_cam_velodyne_geom.transform.translation;
      geometry_msgs::Quaternion quat = transform_cam_velodyne_geom.transform.rotation;
      transform_cam_velodyne.translate(Eigen::Vector3d(position.x, position.y, position.z));
      transform_cam_velodyne.rotate(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
      *tf_cameras_it = transform_cam_velodyne;
      tf_cameras_it++;
    }

    // Go through each point in pointcloud and check if there is a corresponding camera pixel
    for (pcl::PointXYZRGB &point : temp_cloud->points)
    {
      auto tf_cameras_it = tf_cameras.begin();
      for (auto &&cam : cameras_)
      {
        if (cam->size() <= 0)
          break;

        // transform a given point into camera coordinates
        auto temp_point = pcl::transformPoint(point, *tf_cameras_it);
        tf_cameras_it++;

        // obtain a 2d projection of the 3d point
        auto point_in_img =
            (*cam)[0]->getCameraModelPtr()->project3dToPixel(cv::Point3d(temp_point.x, temp_point.y, temp_point.z));

        // check if 2d projection is inside a given camera
        cv::Rect rect(cv::Point(0, 0), (*cam)[0]->image().size());
        if (rect.contains(point_in_img))
        {
          // change RGB values of 3d point with it's 2d projection color
          auto color = (*cam)[0]->image().at<cv::Vec3b>(point_in_img);
          point.b = color[0];
          point.g = color[1];
          point.r = color[2];

          // Break so that multiple cameras don't compete for color of a 3d point
          break;
        }
        else
        {
          point.b = 255;
          point.g = 255;
          point.r = 255;
        }
      }
    }
    temp_cloud->header.frame_id = point_cloud->header.frame_id;
    rgb_cloud_pub_.publish(temp_cloud);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_colorizer");
  boost::shared_ptr<Colorizer> colorizer(new Colorizer());
  ros::spin();
  return 0;
}