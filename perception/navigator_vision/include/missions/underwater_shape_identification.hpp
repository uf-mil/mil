#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <memory>

#include <navigator_vision_lib/image_acquisition/ros_camera_stream.hpp>
#include <navigator_tools.hpp>

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ShapeDetection;

class Shape
{
  using DetectionConstPtr = typename std::unique_ptr<ShapeDetection const>;
  std::string _name;
  std::string _template_path;
  float _pixels_per_meter;
  float _radial_symmetry_angle;
  cv::Mat _template;
public:
  static void loadShapes(std::string directory);
  void load(std::string path);
  std::string name() const { return _name; }
  std::string path() const { return _template_path; }
  float scale() const { return _pixels_per_meter; }
  float radialSymmetryAngle() const { return _radial_symmetry_angle; }
  cv::Mat const * const image() const { return &_template; }
  std::unique_ptr<ShapeDetection const> detect(bool try_rotation=false, int count=10) const;
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
  std::string _template_dir;
  float _template_max_dimension;
  std::map<std::string, Shape> _shapes; 
  float _search_depth;
  float _depth_uncertainty;
  std::map<std::string, float> _detection_thresholds;
  std::map<std::string, std::vector<ShapeDetection>> _detections;
  int _min_detections;
public:
  UnderwaterShapeDetector();
};
