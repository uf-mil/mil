#pragma once

// #include <mil_vision_lib/image_acquisition/camera_model.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>

#include <exception>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>

namespace mil_vision
{
enum class PixelType
{
  /*
      Enumeration for dealing with different image pixel types
      The underlying integers for these enums are compatible with OpenCV's
      "CV_<data_type>C<number_of_channels>"
      macros. The benefit to having these is that we do not have to have OpenCV as a dependency.
      Theoretically, the CameraFrame Objects need not use a cv::Mat and this class' functionality
      would not be affected. The underscore in front of these enums is to solve naming conflicts
      with commonly OpenCV and Eigen macros.
  */
  _8UC1 = 0,
  _8SC1,
  _16UC1,
  _16SC1,
  _32SC1,
  _32FC1,
  _64FC1,
  _8UC2 = 8,
  _8SC2,
  _16UC2,
  _16SC2,
  _32SC2,
  _32FC2,
  _64FC2,
  _8UC3 = 16,
  _8SC3,
  _16UC3,
  _16SC3,
  _32SC3,
  _32FC3,
  _64FC3,
  _8UC4 = 24,
  _8SC4,
  _16UC4,
  _16SC4,
  _32SC4,
  _32FC4,
  _64FC4,
  _UNKNOWN = -1
};

template <typename cam_model_ptr_t = std::shared_ptr<image_geometry::PinholeCameraModel>,
          typename img_scalar_t = uint8_t, typename time_t_ = ros::Time, typename float_t = float>
class CameraFrame
{
  /*
      This class is used to represent a single frame from a camera. It contains information about
      the time the image was taken, the id and camera parameters of the camera that took it and the
      image itself. This is a templated class whose first template parameter is the type of the
      pixel elements (commonly uint8_t for grayscale images and cv::Vec3b for RGB images)
  */

public:
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Constructors and Destructors ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Default constructor for the class. Does not complete any actions; serves as
   * an empty constructor.
   */
  CameraFrame()  // Default Constructor
  {
  }

  /**
   * Copy constructor for the class.
   */
  CameraFrame(const CameraFrame &other)  // Copy Constructor
  {
    this->_seq = other._seq;
    this->_stamp = other._stamp;
    this->_image = other._image.clone();  // Object will have unoque copy of image data
    this->_cam_model_ptr = other.cam_model_ptr;
  }

  /**
   * Constructs a class instance from a ROS message.
   *
   * @param image_msg_ptr A pointer to the image message.
   * @param cam_model_ptr A pointer to the camera model.
   * @param is_rectified Whether the image is rectified (camera geometry has been
   *   taken into account).
   * @param store_at_scale The scale to store the image at.
   */
  CameraFrame(const sensor_msgs::ImageConstPtr &image_msg_ptr, cam_model_ptr_t &cam_model_ptr,
              bool is_rectified = false, float_t store_at_scale = 1.0);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Public Methods /////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Returns a pointer to the camera model.
   *
   * @return The pointer.
   */
  cam_model_ptr_t getCameraModelPtr() const
  {
    return _cam_model_ptr;
  }

  /**
   * Representing the sequence number of camera frames which have come from the same
   * source.
   *
   * These progressively increase over time as more camrea frame objects are generated.
   *
   * @return The sequence number.
   */
  unsigned int seq() const
  {
    return _seq;
  }

  /**
   * Returns the timestamp associated with the image.
   *
   * @return The timestamp.
   */
  time_t_ stamp() const
  {
    return _stamp;
  }

  /**
   * Returns an image mat representing the image.
   *
   * @return The image as an OpenCV mat.
   */
  const cv::Mat_<img_scalar_t> &image() const
  {
    return _image;
  }

  /**
   * Whether the image has been rectified using the disortion parameters specified
   * by the camera model.
   *
   * @return Whether the rectification has occurred.
   */
  bool rectified() const
  {
    return _rectified;
  }

  /**
   * The scale of the image. A scale of 1 represents an image that is its true size
   * and is not scaled.
   *
   * @return The scale.
   */
  float_t getImageScale() const
  {
    return _img_scale;
  }

  /**
   * Copies the image to another OpenCV mat.
   */
  void copyImgTo(cv::Mat dest) const
  {
    dest = image().clone();
  }

  /**
   * Returns true if the camera geometry is NOT specified.
   *
   * @return Whether the camera geometry has been specified.
   */
  bool isCameraGeometryKnown() const
  {
    return _cam_model_ptr == nullptr;
  }

private:
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Private Members ////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // Frames from the same source will have sequentially increasing seq numbers
  unsigned int _seq = 0;

  // Time that this image was taken
  time_t_ _stamp;

  // Stores the image data (not a cv:Mat header pointing to shared image data)
  cv::Mat_<img_scalar_t> _image;

  // Points to a camera model object (shared ownership) that stores information about the intrinsic
  // and extrinsic geometry of the camera used to take this image
  cam_model_ptr_t _cam_model_ptr = nullptr;

  // Identifies if this image has already been rectified with the distortion parameters in the
  // associated camera model object
  bool _rectified = false;

  // Scale of the image compared to that which would be generated by projecting using the camera
  // geometry expressed in the  associated camera model object
  float_t _img_scale = 1.0f;

  // Stores the pixel data type
  mil_vision::PixelType TYPE = mil_vision::PixelType::_UNKNOWN;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Private Methods ////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // void project3DPointToImagePlane(Eigen::Matrix<float_t, 3, 1> cam_frame_pt);
};

///////////////////////////////////////////////////////////////////////////////////////////////////
////// Templated function implementations /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor from ROS image message and ROS pinhole camera model
template <typename cam_model_ptr_t, typename time_t_, typename img_scalar_t, typename float_t>
CameraFrame<cam_model_ptr_t, time_t_, img_scalar_t, float_t>::CameraFrame(
    const sensor_msgs::ImageConstPtr &image_msg_ptr, cam_model_ptr_t &cam_model_ptr, bool is_rectified,
    float_t store_at_scale)
try
{
  // ROS image message decoding
  cv_bridge::CvImageConstPtr _ros_img_bridge;
  std::string encoding = image_msg_ptr->encoding;
  _ros_img_bridge = cv_bridge::toCvShare(image_msg_ptr, encoding);
  _ros_img_bridge->image.copyTo(_image);

  // Resize image as requested
  if (store_at_scale != 1.0)
  {
    cv::resize(_image, _image, cv::Size(0, 0), store_at_scale, store_at_scale);
    this->_img_scale = store_at_scale;
  }

  // Store ptr to cam model object
  this->_cam_model_ptr = cam_model_ptr;

  this->_rectified = is_rectified;

  // Get header information
  _seq = image_msg_ptr->header.seq;
  _stamp = image_msg_ptr->header.stamp;
}
catch (cv_bridge::Exception &e)
{
  ROS_WARN("Error converting sensor_msgs::ImageConstPtr to cv::Mat.");
}
catch (cv::Exception &e)
{
  std::string err_msg = "Error copying cv::Mat created from ROS image message to the cv::Mat stored in the Camera "
                        "Frame Object: " +
                        std::string(e.what());
  ROS_WARN(err_msg.c_str());
  std::cout << "exception caught: " << err_msg << std::endl;
}
catch (const std::exception &e)
{
  ROS_WARN(e.what());
}

}  // namespace mil_vision
