#pragma once

#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <mil_vision_lib/colorizer/common.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>


namespace nav{

struct alignas(16) ColorObservation
{
  using Vec = std::vector<ColorObservation>;
  using VecImg = cv::Mat_<Vec>;
  // float tstamp;   // seconds
  float xyz[3];
  uint8_t bgr[3];
};


class UnoccludedPointsImg
{
/* This class stores the indices of points on a point cloud that are not occluded
   from a given camera view. It quantizes the projection of the point cloud points
   into the image plane into pixel bins and selects the point closest to the center
   of projection to be the only visible one for a given image pixel.
 */

  ColorObservation::VecImg unouccluded_pt_idxs;
  cv::Mat_<float> distance_image;
  PCDPtr<pcl::PointXYZ> cloud_ptr;

public:
  UnoccludedPointsImg();
};

struct PointColorStats
{
  float xyz[3];
  uint8_t bgr[3];
  float var[3];
  int n;
};

}  //namespace nav