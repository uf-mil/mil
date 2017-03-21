#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <memory>
#include <utility>

#include <boost/filesystem.hpp>

#include <navigator_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <navigator_vision_lib/image_filtering.hpp>
#include <navigator_tools.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nav
{

class ShapeDetection;

class Shape
{
  std::string _name;
  std::string _template_path;
  float _pixels_per_meter;
  float _radial_symmetry_angle;
  cv::Mat _template;
  bool _ok = false;
public:
  static std::vector<Shape> loadShapes(std::string directory, float shape_area);
  void load(std::string path, float shape_area);
  std::string name() const { return _name; }
  std::string path() const { return _template_path; }
  float scale() const { return _pixels_per_meter; }
  float radialSymmetryAngle() const { return _radial_symmetry_angle; }
  cv::Mat const * const image() const { return &_template; }
  std::unique_ptr<ShapeDetection const> detect(float threshold, bool try_rotation=false, int count=10) const;
  bool ok() const { return _ok; }
};

struct ShapeDetection
{
  Shape* shape;
  ros::Time time;
  cv::Point2f location;
  float depth;
  float confidence;
  std::shared_ptr<cv::Mat> image;
  cv::RotatedRect roi;
};

class UnderwaterShapeDetector
{
  ros::NodeHandle _nh;
  nav::ROSCameraStream<cv::Vec3b> _camera;
  std::string _ns;
  std::string _template_dir;
  float _shape_area;
  float _search_depth;
  float _depth_uncertainty;
  std::map<std::string, Shape> _shapes; 
  std::map<std::string, bool> _is_target;
  std::map<std::string, float> _detection_thresholds;
  std::map<std::string, std::vector<ShapeDetection>> _detections;
  int _min_detections;
  float _min_boat_displacement;
public:
  UnderwaterShapeDetector(ros::NodeHandle &nh, int img_buf_size, std::string name_space);
  bool searchFor(std::vector<std::string> target_shapes);
  void calibrateThresholds();
};

}  // namespace nav
