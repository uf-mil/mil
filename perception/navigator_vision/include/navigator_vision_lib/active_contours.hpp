#pragma once

#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nav
{

class SegmentDescription;
class  ClosedCurve;

namespace Perturbations
{
  struct Perturbation
  {
    std::vector<uint8_t> pt_idxs;
    std::vector<cv::Point2i> toPointSequence(cv::Point2i center);
  };
  
  static std::map<std::pair<uint8_t, uint8_t>, std::vector<std::vector<uint8_t>>> perturbation_cache;
  //static std::vector<std::vector<std::vector<std::vector<uint8_t>>>> perturbation_cache;
  
  void initPerturbationCache();
  
//  std::vector<Perturbation>* getPerturbations(const SegmentDescription& seg_desc);

  std::vector<std::vector<uint8_t>> getPerturbations(uint8_t entry, uint8_t exit);
  uint8_t getIdxFromPoint(cv::Point2i hood_point);
  cv::Point2i getPointFromIdx(uint8_t idx);
  std::vector<uint8_t> getHoodIdxs(uint8_t idx, bool include_border);
  bool isNeighbor(uint8_t idx1, uint8_t idx2);
  void growRoute(const std::vector<uint8_t>& partial, const std::vector<uint8_t>& occupied,
                 uint8_t entry, uint8_t exit);
}  // namespace Perturbations


class ClosedCurve
{
  std::vector<cv::Point2i> _curve_points;

public:
  ClosedCurve(std::vector<cv::Point2i> points);
  ClosedCurve applyPerturbation(Perturbations::Perturbation perturb);
};

class CurvePerturbation
{
  std::vector<int> perturbed_segment;
  uint8_t entry;
  uint8_t exit;
};

class ActiveContour
{
  ClosedCurve _contour;

public:
  ActiveContour();
};
  
}  // namespace nav
