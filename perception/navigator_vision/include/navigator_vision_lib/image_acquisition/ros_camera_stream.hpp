#pragma once

#include <navigator_vision_lib/image_acquisition/camera_frame_sequence.hpp>
#include <navigator_tools.hpp>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>

#include <cmath>
#include <string>
#include <mutex>

// DBG
#include <iostream>


namespace nav
{

template<typename img_scalar_t = uint8_t, typename float_t = float>
class ROSCameraStream : public CameraFrameSequence<std::shared_ptr<image_geometry::PinholeCameraModel>, img_scalar_t, ros::Time, float_t>
{
  using cam_model_ptr_t = std::shared_ptr<image_geometry::PinholeCameraModel>;
  using time_t_ = ros::Time;

public:

  // Type aliases
  using CamFrame = CameraFrame<cam_model_ptr_t, img_scalar_t, time_t_, float_t>;
  using CamFramePtr = std::shared_ptr<CamFrame>;
  using CamFrameConstPtr = std::shared_ptr<CamFrame const>;
  using CamFrameSequence = CameraFrameSequence<cam_model_ptr_t, img_scalar_t, time_t_, float_t>;
  using CircularBuffer = boost::circular_buffer<CamFramePtr>;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Constructors and Destructors ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // Default Constructor
  ROSCameraStream(ros::NodeHandle nh, size_t buffer_size)
  : _frame_ptr_circular_buffer(buffer_size), _nh(nh), _it(_nh) {}

  ~ROSCameraStream();

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Public Methods /////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  
  // Returns true if the camera stream object has been successfully initialized
  bool init(std::string &camera_topic);

  // Returns false if an internal error has ocurred or ROS is in the process of shutting down the
  // enclosing node
  bool ok() {return _ok && ros::ok();};

  typename CircularBuffer::iterator begin() const
  { 
    return _frame_ptr_circular_buffer.begin();
  }

  typename CircularBuffer::iterator end() const
  {
    return _frame_ptr_circular_buffer.end();
  }

  size_t size() const
  {
    return _frame_ptr_circular_buffer.size();
  }

  cam_model_ptr_t getCameraModelPtr() const
  {
    return _cam_model_ptr;  // returns nullptr if geometry is unknown
  }
  
  // If geometry is constant then calls to rows, or calls will be valid
  bool isGeometryConst() const
  {
    return true;
  }
  
  int rows() const
  {
    return ROWS;
  }

  int cols() const
  {
    return COLS;
  }

  CamFrameConstPtr getFrameFromTime(ros::Time desired_time);

  CamFrameConstPtr operator[](int i);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Public Members /////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

private:

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Private Methods ////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  void _addFrame(CamFramePtr &new_frame_ptr);

  void _newFrameCb(const sensor_msgs::ImageConstPtr &image_msg_ptr);

  
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Private Members ////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // Container for all included CameraFrame objects
  CircularBuffer _frame_ptr_circular_buffer;

  // cv::Ptr to a shared CameraInfo object for all frames in the sequence. If null, it indicates
  // that the Frames have different CameraInfo objects which should be examined individually
  cam_model_ptr_t _cam_model_ptr = nullptr;

  // Time bounds of buffer
  time_t_ _start_time {};
  time_t_ _end_time {};

  // Shared CameraFrame properties if camera geometry is constant
  int COLS = -1;
  int ROWS = -1;
  nav::PixelType TYPE = nav::PixelType::_UNKNOWN;

  // Mutex for multithreaded access to camera frame data
  std::mutex _mtx;

  // ROS node handle
  ros::NodeHandle _nh;

  // ROS image transportg
  image_transport::ImageTransport _it;

  // Custom ROS Callback Queue
  ros::CallbackQueue _cb_queue;

  // Flexible topic subscription with potential for filtering and chaining
  message_filters::Subscriber<sensor_msgs::Image> img_sub;

  // ROS spinner to handle callbacks in a background thread
  ros::AsyncSpinner async_spinner {1, &_cb_queue};

  // The ros topic we will subscribe to
  std::string _img_topic = "uninitialized";

  // Status Flag
  bool _ok = false;

  // Store error messages here
  std::string _err_msg{""};
};

///////////////////////////////////////////////////////////////////////////////////////////////////
////// Templated function implementations /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

// Initializer for when there is an image msg being published w/ a corresponding camera info msg
template<typename img_scalar_t, typename float_t>
bool ROSCameraStream<img_scalar_t, float_t>::init(std::string &camera_topic)
{
  using nav::tools::operator "" _s;  // convert raw string literal to std::string
  _img_topic = camera_topic;
  bool success = false;
  image_transport::CameraSubscriber cam_sub;

  // subscribes to image msg and camera info and initializes CameraModel Object then disconnets subscriber
  auto init_lambda = 
    [&](const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) mutable
    {
      std::string init_msg {"ROSCameraStream: Initializing with "};
      init_msg += _img_topic;
      // ROS_INFO_NAMED("ROSCameraStream", init_msg.c_str());
      // Set ptr to camera model object
      std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_ptr{new image_geometry::PinholeCameraModel()};
      camera_model_ptr->fromCameraInfo(info_msg_ptr);
      this->_cam_model_ptr = camera_model_ptr;

      // Set metadata attributes
      this->ROWS = image_msg_ptr->height;
      this->COLS = image_msg_ptr->width;
      success = true;
      cam_sub.shutdown();
      return;
    };

  // Subscribe to both camera topic and camera info topic
  cam_sub = _it.subscribeCamera(_img_topic, 100, init_lambda);

  // The amount of time that we will try to wait for a cb from cam_sub
  ros::WallDuration timeout{5, 0};

  // Loop to check if we get callbacks from cam_sub
  auto sub_start = ros::WallTime::now();
  ros::WallRate rate{10};  // 10 HZ so we dont spam cpu
  while(ros::WallTime::now() - sub_start < timeout)
  {
    ros::spinOnce();
    if(success)
    {
      // Subscribe to ROS image topic
      img_sub.subscribe(_nh, _img_topic, 100, ros::TransportHints(), &_cb_queue);

      // Register callback to process frames published on camera img topic
      auto img_cb = [this](const sensor_msgs::ImageConstPtr &image_msg_ptr){_newFrameCb(image_msg_ptr);};
      img_sub.registerCallback(img_cb);

      // Start listening for image messages in a background thread
      async_spinner.start();

      _ok = true;
      return _ok;  // Ideal exit point for init
    }
    rate.sleep();
  }

  _err_msg = "ROSCameraStream: Timed out trying to initialize with camera topic "_s + _img_topic
    + ". Run 'rostopic echo "_s + _img_topic + "' to make sure it is being published to.";
  ROS_WARN_NAMED("ROSCameraStream", _err_msg.c_str());
  return _ok;
}
// TODO: handle construction from img msg when there is no matching camera info msg

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

template<typename img_scalar_t, typename float_t>
typename ROSCameraStream<img_scalar_t, float_t>::CamFrameConstPtr
ROSCameraStream<img_scalar_t, float_t>::getFrameFromTime(ros::Time desired_time)
{
  using nav::tools::operator "" _s;

  // Check bounds on time
  if(desired_time < this->_start_time || desired_time > this->_end_time)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(1, "ROSCameraStream",
      "ROSCameraStream: The camera frame you requested is outside the time range of the buffer.");
    return nullptr;
  }

  CamFrameConstPtr closest_in_time = this->operator[](0);
  double min_abs_nsec_time_diff = fabs((this->operator[](0)->stamp() - desired_time).toNSec());
  double abs_nsec_time_diff = -1;
  for(CamFrameConstPtr frame_ptr : this->_frame_ptr_circular_buffer)
  {
    abs_nsec_time_diff = fabs((frame_ptr->stamp() - desired_time).toNSec());
    if(abs_nsec_time_diff < min_abs_nsec_time_diff)
    {
      closest_in_time = frame_ptr;
      min_abs_nsec_time_diff = abs_nsec_time_diff;
    }
  }
  return closest_in_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

template<typename img_scalar_t, typename float_t>
typename ROSCameraStream<img_scalar_t, float_t>::CamFrameConstPtr 
ROSCameraStream<img_scalar_t, float_t>::operator[](int i)
{
  using nav::tools::operator "" _s;

  // Prevent adding new frames while frames are being accessed
  _mtx.lock();

  // shared_ptr to a dynamically allocated reference frame object
  CamFrameConstPtr shared_ptr_to_const_frame;

  try
  {
    if(i >= 0) // regular access for non-negative indices
    {
      shared_ptr_to_const_frame = this->_frame_ptr_circular_buffer[i];
    }
    else{  // reverse access for negative indices (ex. [-1] refers to the last element)
      size_t past_last_idx = this->_frame_ptr_circular_buffer.end() - this->_frame_ptr_circular_buffer.begin(); // DBG
      shared_ptr_to_const_frame =  this->_frame_ptr_circular_buffer[past_last_idx + i];  // DBG
    }
  }
  catch(std::exception &e)
  {
    auto err = "ROSCameraStream: The circular buffer index you are trying to acess is out of bounds:\n"_s + e.what();
    ROS_WARN_THROTTLE_NAMED(1, "ROSCameraStream",  err.c_str());
    return nullptr;
  }

  _mtx.unlock();
  return shared_ptr_to_const_frame;
}

template<typename img_scalar_t, typename float_t>
void 
ROSCameraStream<img_scalar_t, float_t>::_addFrame(CamFramePtr &new_frame_ptr)
{
  // Prevent accessing frames while new frames are being added
  _mtx.lock();
  this->_frame_ptr_circular_buffer.push_front(new_frame_ptr);
  _mtx.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

template<typename img_scalar_t, typename float_t>
void ROSCameraStream<img_scalar_t, float_t>::_newFrameCb(const sensor_msgs::ImageConstPtr &image_msg_ptr)
{
  // Check if the topic name contains the string "rect"
  bool rectified = _img_topic.find(std::string("rect")) != std::string::npos;

  // Create shared pointer to dynamically allocated Camera Frame object constructed from ros img msg
  CamFramePtr new_frame_ptr{new CamFrame(image_msg_ptr, this->_cam_model_ptr, rectified, 1.0)};

  // Add shared pointer to CameraFrame object to the circular buffer
  _addFrame(new_frame_ptr);

  // Update the bounding timestamps for the frame sequence
  this->_start_time = image_msg_ptr->header.stamp;
  this->_end_time = this->_frame_ptr_circular_buffer.back()->stamp();
}

template<typename img_scalar_t, typename float_t>
ROSCameraStream<img_scalar_t, float_t>::~ROSCameraStream()
{
  img_sub.unsubscribe();
  _cb_queue.clear();
  _cb_queue.disable();
}

} // namespace nav