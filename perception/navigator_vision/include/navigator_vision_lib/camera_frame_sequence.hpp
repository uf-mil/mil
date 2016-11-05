#pragma once

#include <camera_frame.hpp>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

namespace nav
{

template<typename cam_model_ptr_t = boost::shared_ptr<image_geometry::PinholeCameraModel>,
         typename time_t_ = ros::Time,
         typename img_scalar_t = uint8_t,
         typename float_t = float>
class CameraFrameSequence
{
/*
	This class is used to store and operate on camera frame sequences (videos). It contains methods
	to make it more convenient to create visual algorithms that take into account temporal 
	information. 
	It also has convenient constructors to construct these sequences by subscribing to
	ROS camera topics, by taking in frames from a v4l camera or by, loading frames from a saved 
	video file.
	It is also possible to draw on contained camera frames as a batch.
*/

// Type Alises
using CircularBuffer = boost::circular_buffer<boost::shared_ptr<CameraFrame<cam_model_ptr_t, time_t_, img_scalar_t, float_t>>>;

public:

    ///////////////////////////////////////////////////////////////////////////////////////////////
	// Constructors and Destructors ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

	// Default Constructor
	CameraFrameSequence(size_t capacity) : _frame_ptr_circular_buffer(capacity)
	{
	}

	// TODO: implement copy and move constructors

	~CameraFrameSequence()
	{
	}


    ///////////////////////////////////////////////////////////////////////////////////////////////
	// Public Methods /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

	virtual typename CircularBuffer::iterator begin()
	{
	 return _frame_ptr_circular_buffer.begin();
	}

	virtual typename CircularBuffer::iterator end()
	{
		return _frame_ptr_circular_buffer.end();
	}

    size_t length() const
    {
    	return _frame_ptr_circular_buffer.size();
    }

    // Accessors

    // Returns a copy of the CameraFrame taken closest to the given time
    virtual boost::shared_ptr<CameraFrame<cam_model_ptr_t, time_t_, img_scalar_t, float_t> const>
    getFrameFromTime(time_t_) = 0;

    // Returns reference to the nth frame from the most recent. For example frame_sequence[0] is 
    // the most recent frame, frame_sequence[-1] is the oldest frame, and frame_sequence[-2] is 
    // the second oldest frame
    virtual boost::shared_ptr<CameraFrame<cam_model_ptr_t, time_t_, img_scalar_t, float_t> const>
    operator[](int) = 0;


    ///////////////////////////////////////////////////////////////////////////////////////////////
	// Public Members /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

protected:

    ///////////////////////////////////////////////////////////////////////////////////////////////
	// Protected Methods //////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    virtual void _addFrame(boost::shared_ptr<CameraFrame<cam_model_ptr_t, time_t_, img_scalar_t, float_t>> &new_frame_ptr) = 0;
	
    ///////////////////////////////////////////////////////////////////////////////////////////////
	// Private Members ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Container for all included CameraFrame objects
    CircularBuffer _frame_ptr_circular_buffer;

    // cv::Ptr to a shared CameraInfo object for all frames in the sequence. If null, it indicates
    // that the Frames have different CameraInfo objects which should be examined individually
    cam_model_ptr_t _cam_model_ptr;

    // Shared CameraFrame properties
   	int COLS = -1;
   	int ROWS = -1;
   	nav::PixelType TYPE = nav::PixelType::_UNKNOWN;

   	bool CONST_CAM_GEOMETRY = false;
   	bool KNOWN_CAM_GEOMETRY = false;

   	time_t_ _start_time {};
   	time_t_ _end_time {};
};

}  // namespace nav