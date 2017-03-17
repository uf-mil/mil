#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace nav{

static const double PI = 3.1415926535897932;

namespace tools
{

// Returns a vector of topic names (std::string) that end with "image_rect_color"
// if color is false then it looks for those that end in "image_rect"
std::vector<std::string> getRectifiedImageTopics(bool color = true);

// converts raw string literals to std:string's
inline std::string operator "" _s(const char* str, size_t /*length*/) 
{ 
    return std::string(str); 
}


// Sorts contours by curve length
void sortContours(std::vector<std::vector<cv::Point>>& contour_vec, bool ascending=true);

}  // namespace nav::tools
}  // namespace nav
