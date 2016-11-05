#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>

namespace nav{
namespace tools{

// Returns a vector of topic names (std::string) that end with "image_rect_color"
// if color is false then it looks for those that end in "image_rect"
std::vector<std::string> getRectifiedImageTopics(bool color = true);

// converts raw string literals to std:string's
inline std::string operator "" _s(const char* str, size_t /*length*/) 
{ 
    return std::string(str); 
}

}  // namespace nav::tools
}  // namespace nav