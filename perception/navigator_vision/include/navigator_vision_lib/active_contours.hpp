#pragma once

#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <functional>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nav
{

class SegmentDescription;
class ClosedCurve;

namespace Perturbations
{
  static std::map<std::pair<uint8_t, uint8_t>, std::vector<std::vector<uint8_t>>> perturbation_cache;
  void initPerturbationCache();
  std::vector<std::vector<uint8_t>> getPerturbations(uint8_t entry, uint8_t exit);
  uint8_t getIdxFromPoint(cv::Point2i hood_point);
  cv::Point2i getPointFromIdx(uint8_t idx);
  std::vector<cv::Point2i> getPointList(std::vector<uint8_t>& idx_list);
  std::vector<uint8_t> getHoodIdxs(uint8_t idx, bool include_border);
  bool isNeighbor(uint8_t idx1, uint8_t idx2);
  bool isNeighborPoint(const cv::Point2i &pt1, const cv::Point2i &pt2);
  void growRoute(const std::vector<uint8_t>& partial, const std::vector<uint8_t>& occupied,
                 uint8_t entry, uint8_t exit);
  std::vector<cv::Point2i> perturb(const std::vector<cv::Point2i>& src_curve, std::vector<uint8_t> perturbation,int idx);
}  // namespace Perturbations

class ClosedCurve
{
  std::vector<cv::Point2i> _curve_points;

public:
  struct Perturbation
  {
    size_t idx;
    uint8_t entry;
    uint8_t exit;
    std::vector<uint8_t> route;
  };

  ClosedCurve(std::vector<cv::Point2i> points);
  void applyPerturbation(const std::vector<uint8_t>& perturbation, int idx);
  ClosedCurve perturb(const std::vector<uint8_t>& perturbation, int idx);
  static bool validateCurve(std::vector<cv::Point2i>& curve);
  std::vector<float> calcCosts(const cv::Mat& img, std::vector<Perturbation> candidate_perturbs,
                               std::function<float(const cv::Mat&, Perturbation)> cb);
};

class ActiveContour
{
  ClosedCurve _contour;

public:
  ActiveContour();
};
  
}  // namespace nav
