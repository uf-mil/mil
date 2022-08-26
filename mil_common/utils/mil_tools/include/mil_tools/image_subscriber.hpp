#ifndef IMAGE_SUBSCRIBER_H_
#define IMAGE_SUBSCRIBER_H_

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Time.h>

#include <boost/optional.hpp>
#include <functional>
#include <opencv2/opencv.hpp>
#include <type_traits>

class ImageSubscriber
{
public:
  template <typename T>
  ImageSubscriber(const ros::NodeHandle& nh_, const std::string& topic, void (T::*func)(cv::Mat&), T* a,
                  const std::string& encoding = "bgr8", int queue_size = 1)
    : encoding_(encoding), queue_size_(queue_size), it_(nh_)
  {
    image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallback, this);
    function_ = std::bind(func, a, std::placeholders::_1);
    camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::infoCallback, this);
  }

  ImageSubscriber(const ros::NodeHandle& nh_, const std::string& topic, void (*func)(cv::Mat&),
                  const std::string& encoding = "bgr8", int queue_size = 1)
    : encoding_(encoding), queue_size_(queue_size), it_(nh_)
  {
    image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallback, this);
    function_ = func;
    camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::infoCallback, this);
  }

  std::string getInfoTopic(const std::string& input)
  {
    int location = input.rfind('/');
    if (location == input.length() - 1)
      location = input.rfind('/', input.length() - 2);
    return input.substr(0, location - 1) + "/camera_info";
  }

  boost::optional<sensor_msgs::CameraInfo> waitForCameraInfo(unsigned int timeout = 10)
  {
    ROS_WARN("Blocking -- waiting at most %d seconds for camera info.", timeout);

    auto time = ros::Duration(timeout);
    time.sleep();
    auto start_time = ros::Time::now();

    while (ros::Time::now() - start_time < time && ros::ok())
    {
      if (camera_info_.has_value())
      {
        ROS_INFO("Camera info found!");
        return camera_info_.get();
      }
      ros::Duration(0.2).sleep();
    }

    ROS_ERROR("Camera info not found.");
    return boost::none;
  }

  bool waitForCameraModel(image_geometry::PinholeCameraModel& pinhole_camera, unsigned int timeout = 10)
  {
    auto msg = waitForCameraInfo(timeout);
    if (!msg)
      return false;
    pinhole_camera.fromCameraInfo(msg);
    return true;
  }

  void infoCallback(const sensor_msgs::CameraInfo& info)
  {
    camera_info_sub_.shutdown();
    camera_info_.emplace(info);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    try
    {
      last_image_time_ = image.header.stamp;
      function_(cv_bridge::toCvCopy(image, encoding_)->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s'.", image->encoding, encoding_);
    }
  }

  boost::optional<std_msgs::Time> getLastImageTime()
  {
    return camera_info_;
  }

private:
  std::string encoding_;
  int queue_size_;

  std::function<void(cv::Mat&)> function_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;

  ros::Subscriber camera_info_sub_;
  boost::optional<sensor_msgs::CameraInfo> camera_info_;
  boost::optional<std_msgs::Time> last_image_time_;
};
#endif  // IMAGE_SUBSCRIBER_H_
