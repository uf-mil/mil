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

/**
 * Abstraction for subscribing to image outputs and their info.
 * This returns a copy or reference of the image depending on what you specify in the constructor.
 * The camera info contains valuable information such as the camera matrix and will exit gracefully if not found.
 */
class ImageSubscriber
{
public:
  /**
   * Create an ImageSubscriber using a class member function as the function callback.
   * @param nh NodeHandle for the node that is subscribing to a camera topic.
   * @param topic Contains the camera topic to subscribe to.
   * @param func Function pointer to class member callback function.
   * @param object Pointer to object that function belongs to.
   * @param use_copy Whether to use a copy or reference of the converted image. If unsure use copy.
   * @param encoding Type of encoding for camera topic.
   * @param queue_size Queue size to use for subscriber.
   */
  template <typename T>
  ImageSubscriber(const ros::NodeHandle& nh, const std::string& topic, void (T::*func)(cv::Mat&), T* object,
                  bool use_copy = true, const std::string& encoding = "bgr8", int queue_size = 1)
    : encoding_(encoding), queue_size_(queue_size), it_(nh)
  {
    if (use_copy)
    {
      image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallbackCopy, this);
    }
    else
    {
      image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallbackReference, this);
    }
    function_ = std::bind(func, object, std::placeholders::_1);
    camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::infoCallback, this);
  }

  /**
   * Create an ImageSubscriber using a normal function as the function callback.
   * @param nh NodeHandle for the node that is subscribing to a camera topic.
   * @param topic Contains the camera topic to subscribe to.
   * @param func Function pointer to callback function.
   * @param use_copy Whether to use a copy or reference of the converted image. If unsure use copy.
   * @param encoding Type of encoding for camera topic.
   * @param queue_size Queue size to use for subscriber.
   */
  ImageSubscriber(const ros::NodeHandle& nh, const std::string& topic, void (*func)(cv::Mat&), bool use_copy = true,
                  const std::string& encoding = "bgr8", int queue_size = 1)
    : encoding_(encoding), queue_size_(queue_size), it_(nh)
  {
    if (use_copy)
    {
      image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallbackCopy, this);
    }
    else
    {
      image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallbackReference, this);
    }
    function_ = func;
    camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::infoCallback, this);
  }

  /**
   * Safely returns the camera info if it is found. Waits for timeout seconds to receive the camera info.
   * @param timeout Time in seconds to wait for camera info to be received.
   * @return optional value to return data if available otherwise nothing if not.
   */
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

  /**
   * Waits for camera info to be received and fills in the given PinholeCameraModel with the data from it.
   * @param pinhole_camera PineholeCameraModel to store the resulting camera model in.
   * @param timeout Time in seconds to wait for camera info to be received.
   * @return true if data is received, false otherwise.
   */
  bool waitForCameraModel(image_geometry::PinholeCameraModel& pinhole_camera, unsigned int timeout = 10)
  {
    auto msg = waitForCameraInfo(timeout);
    if (!msg)
      return false;
    pinhole_camera.fromCameraInfo(msg);
    return true;
  }

  /**
   * Function callback for camera info.
   * @param info Camera info message from ROS.
   */
  void infoCallback(const sensor_msgs::CameraInfo& info)
  {
    camera_info_sub_.shutdown();
    camera_info_.emplace(info);
  }

  /**
   * Function callback for the image copy.
   * @param image Image message from ROS.
   */
  void imageCallbackCopy(const sensor_msgs::ImageConstPtr& image)
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

  /**
   * Function callback for the image reference.
   * @param image Image message from ROS.
   */
  void imageCallbackReference(const sensor_msgs::ImageConstPtr& image)
  {
    try
    {
      last_image_time_ = image.header.stamp;
      function_(cv_bridge::toCvShare(image, encoding_)->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s'.", image->encoding, encoding_);
    }
  }

  /**
   * Returns the timestamp for the last image.
   * @return optional value containing the time the last image was received.
   */
  boost::optional<std_msgs::Time> getLastImageTime()
  {
    return camera_info_;
  }

private:
  /**
   * Parses the inputted topic to get the camera_info topic assuming standard pattern is followed.
   * @param input Camera topic
   * @return camera info topic
   */
  std::string getInfoTopic(const std::string& input)
  {
    int location = input.rfind('/');
    if (location == input.length() - 1)
      location = input.rfind('/', input.length() - 2);
    return input.substr(0, location - 1) + "/camera_info";
  }

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
