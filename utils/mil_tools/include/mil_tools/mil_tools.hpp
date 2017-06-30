#pragma once

#include <ros/ros.h>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

namespace mil_tools
{
static const double PI = 3.1415926535897932;

// Returns a vector of topic names (std::string) that end with "image_rect_color"
// if color is false then it looks for those that end in "image_rect"
std::vector<std::string> getRectifiedImageTopics(bool color = true);

// converts raw string literals to std:string's
inline std::string operator"" _s(const char* str, size_t /*length*/)
{
  return std::string(str);
}

// Sorts contours by curve length
void sortContours(std::vector<std::vector<cv::Point>>& contour_vec, bool ascending = true);

// nCk: Combinations of k elements from a set of size n (indexes)
void combinations(uint8_t n, uint8_t k, std::vector<std::vector<uint8_t>>& idx_array);
// Helper function for combinations
void _increase_elements_after_level(std::vector<uint8_t> comb, std::vector<std::vector<uint8_t>>& comb_array, uint8_t n,
                                    uint8_t k, uint8_t level);
}
