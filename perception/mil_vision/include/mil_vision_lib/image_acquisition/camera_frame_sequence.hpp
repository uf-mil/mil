#pragma once

#include <mil_vision_lib/image_acquisition/camera_frame.hpp>
#include <boost/circular_buffer.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <vector>
#include <memory>

namespace nav
{

template<typename cam_model_ptr_t, typename img_scalar_t, typename time_t_,typename float_t = float>
class CameraFrameSequence
{
/*
  This is an abstract class interface for classes that represent sequences of images. It provides
  basic functionality to access individual frames along with camera geometry information. It is 
  intended to make it more convenient to create visual algorithms that take into account temporal 
  information.
*/

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

  CameraFrameSequence() = default;

  CameraFrameSequence(const CameraFrameSequence&) = delete;  // Forbid copy construction

  CameraFrameSequence(CameraFrameSequence&&) = delete;  // Forbid move construction

  virtual ~CameraFrameSequence() = default;


  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Public Methods /////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  virtual size_t size() const = 0;

  virtual cam_model_ptr_t getCameraModelPtr() const = 0;  // returns nullptr if geometry is unknown
  
  // If geometry is constant then calls to rows, or calls will be valid
  virtual bool isGeometryConst() const = 0;
  
  virtual int rows() const = 0;

  virtual int cols() const = 0;

  // Accessors

  // Returns a copy of the CameraFrame taken closest to the given time
  virtual CamFrameConstPtr getFrameFromTime(time_t_) = 0;

  // Returns reference to the nth frame from the most recent. For example frame_sequence[0] is 
  // the most recent frame, frame_sequence[-1] is the oldest frame, and frame_sequence[-2] is 
  // the second oldest frame
  virtual CamFrameConstPtr operator[](int) = 0;

};

}  // namespace nav