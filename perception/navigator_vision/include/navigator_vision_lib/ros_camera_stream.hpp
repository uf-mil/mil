#pragma once

#include <camera_frame_sequence.hpp>
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <boost/circular_buffer.hpp>
#include <cmath>

// DBG
#include <iostream>


namespace nav
{

template<typename img_scalar_t = uint8_t, typename float_t = float>
class ROSCameraStream : public CameraFrameSequence<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>
{

// Type Aliases
using CamFrame = CameraFrame<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>;
using CamFramePtr = boost::shared_ptr<CameraFrame<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>>;
using CamFrameConstPtr = boost::shared_ptr<CameraFrame<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t> const>;
using CamFrameSequence = CameraFrameSequence<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>;


public:

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Default Constructor
    ROSCameraStream(ros::NodeHandle nh, size_t buffer_size)
    : CamFrameSequence(buffer_size),
      _it(nh)
    {
        this->_nh = nh;
        this->_capacity = buffer_size;
    }

    ~ROSCameraStream();

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Public Methods /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    bool init(std::string camera_topic);

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

    // Initialization flag
    bool _initialized = false;

    // Mutex for multithreaded to camera frame data
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

    // The maximum amount of frames that this object will hold at one time
    size_t _capacity = -1;

    // The ros topic we will subscribe to
    std::string _img_topic = "uninitialized";
};

///////////////////////////////////////////////////////////////////////////////////////////////////
////// Templated function implementations /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor for when there is an image msg being published w/ a corresponding camera info msg
template<typename img_scalar_t, typename float_t>
bool ROSCameraStream<img_scalar_t, float_t>::init(std::string camera_topic)
{
    this->_img_topic = camera_topic;
    bool success = false;
    image_transport::CameraSubscriber cam_sub;

    // subscribes to image msg and camera info and initializes CameraModel Object then disconnets subscriber
    auto init_lambda = 
        [&](const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) mutable
        {
            ROS_WARN("Initializing ROSCameraStream object.");
            // Set ptr to camera model object
            boost::shared_ptr<image_geometry::PinholeCameraModel> camera_model_ptr{new image_geometry::PinholeCameraModel()};
            camera_model_ptr->fromCameraInfo(info_msg_ptr);
            this->_cam_model_ptr = camera_model_ptr;

            // Set metadata attributes
            this->ROWS = image_msg_ptr->height;
            this->COLS = image_msg_ptr->width;
            this->CONST_CAM_GEOMETRY = true;
            this->KNOWN_CAM_GEOMETRY = true;
            success = true;
            cam_sub.shutdown();
            ROS_WARN("ROSCameraStream object initialized.");
            return;
        };

    // Subscribe to both camera topic and camera info topic
    cam_sub = _it.subscribeCamera(camera_topic, 100, init_lambda);

    // The amount of time that we will try to wait for a cb from cam_sub
    ros::Duration timeout{5, 0};
    auto sub_start = ros::Time::now();

    // Loop to check if we get callbacks from cam_sub
    ros::Rate rate{10};  // 10 HZ so we dont spam cpu

    while(ros::Time::now() - sub_start < timeout)
    {
        ros::spinOnce();
        if(success)
        {
            // Subscribe to ROS image topic
            img_sub.subscribe(_nh, this->_img_topic, 100, ros::TransportHints(), &_cb_queue);

            // Register callback to process frames published on camera img topic
            auto img_cb = [this](const sensor_msgs::ImageConstPtr &image_msg_ptr){this->_newFrameCb(image_msg_ptr);};
            img_sub.registerCallback(img_cb);

            // Start listening for image messages in a background thread
            async_spinner.start();

            this->_initialized = true;
            return true;
        }
        rate.sleep();
    }

    std::string err_msg {"timeout: failed to initialize ROSCameraStream object with camera topic '"};
    err_msg += camera_topic + "'.";
    ROS_WARN(err_msg.c_str());
    this->_initialized = false;
    return false;
}
// TODO: handle construction from img msg when there is no matching camera info msg

template<typename img_scalar_t, typename float_t>
typename ROSCameraStream<img_scalar_t, float_t>::CamFrameConstPtr
ROSCameraStream<img_scalar_t, float_t>::getFrameFromTime(ros::Time desired_time)
{
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

template<typename img_scalar_t, typename float_t>
typename ROSCameraStream<img_scalar_t, float_t>::CamFrameConstPtr 
ROSCameraStream<img_scalar_t, float_t>::operator[](int i)
{
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
            size_t past_last_idx = this->_frame_ptr_circular_buffer.end() - this->_frame_ptr_circular_buffer.begin();
            shared_ptr_to_const_frame =  this->_frame_ptr_circular_buffer[past_last_idx + i];
        }
    }
    catch(std::exception &e)
    {
        std::string err_msg = "The circular buffer index you are trying to acess is out of bounds:\n" + std::string(e.what());
        ROS_WARN(err_msg.c_str());
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

template<typename img_scalar_t, typename float_t>
void ROSCameraStream<img_scalar_t, float_t>::_newFrameCb(const sensor_msgs::ImageConstPtr &image_msg_ptr)
{
    // Check if the topic name contains the string "rect"
    bool rectified = this->_img_topic.find(std::string("rect")) != std::string::npos;

    // Create shared pointer to dynamically allocated Camera Frame object constructed from ros img msg
    boost::shared_ptr<CameraFrame<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>> new_frame_ptr 
        {new CameraFrame<boost::shared_ptr<image_geometry::PinholeCameraModel>, ros::Time, img_scalar_t, float_t>
            (image_msg_ptr, this->_cam_model_ptr, rectified, 1.0)};

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