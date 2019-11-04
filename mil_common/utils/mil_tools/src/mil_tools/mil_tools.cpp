#include <mil_tools/mil_tools.hpp>
#include <mil_tools/msg_helpers.hpp>
#include <mil_tools/param_helpers.hpp>

namespace mil_tools
{
using namespace std;

vector<string> getRectifiedImageTopics(bool color)
{
  // get all currently published topics from master
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  // Define lambda to determine if a topic is a rectified img topic
  string target_pattern;
  target_pattern = color ? "image_rect_color" : "image_rect";
  auto isRectImgTopic = [&target_pattern](ros::master::TopicInfo topic_info) {
    // Find last slash
    size_t final_slash = topic_info.name.rfind('/');

    // Match end of topic name to image_rect pattern
    if (final_slash == string::npos)
      return false;
    else
    {
      string topic_name_end = topic_info.name.substr(final_slash + 1);
      return (topic_name_end.find(target_pattern) != string::npos) ? true : false;
    }
  };  // end lambda isRectImgTopic

  // return list of rectified image topics
  vector<string> image_rect_topics;
  for (ros::master::TopicInfo topic : master_topics)
  {
    if (isRectImgTopic(topic))
      image_rect_topics.push_back(topic.name);
  }
  return image_rect_topics;
}

void sortContours(vector<vector<cv::Point2i>> &contour_vec, bool ascending)
{
  auto comp_length = ascending ?
                         [](vector<cv::Point2i> const &contour1, vector<cv::Point2i> const &contour2) {
                           return fabs(arcLength(contour1, true)) < fabs(arcLength(contour2, true));
                         } :
                         [](vector<cv::Point2i> const &contour1, vector<cv::Point2i> const &contour2) {
                           return fabs(arcLength(contour1, true)) > fabs(arcLength(contour2, true));
                         };

  sort(contour_vec.begin(), contour_vec.end(), comp_length);
}

void combinations(uint8_t n, uint8_t k, std::vector<std::vector<uint8_t>> &idx_array)
{
  idx_array = std::vector<std::vector<uint8_t>>();

  std::vector<uint8_t> first_comb;

  // set first combination indices
  for (uint8_t i = 0; i < k; i++)
  {
    first_comb.push_back(i);
  }

  uint8_t level = 0;

  _increase_elements_after_level(first_comb, idx_array, n, k, level);
}

void _increase_elements_after_level(std::vector<uint8_t> comb, std::vector<std::vector<uint8_t>> &comb_array, uint8_t n,
                                    uint8_t k, uint8_t level)
{
  std::vector<uint8_t> parent = comb;
  std::vector<std::vector<uint8_t>> children;

  while (true)
  {
    for (uint8_t idx = level; idx < k; idx++)
    {
      comb[idx] = comb[idx] + 1;
    }
    if (comb[level] > n - (k - level))
      break;
    children.push_back(comb);
  }

  if (level == k - 1)
  {
    comb_array.push_back(parent);
    for (std::vector<uint8_t> child : children)
    {
      comb_array.push_back(child);
    }
  }
  else
  {
    _increase_elements_after_level(parent, comb_array, n, k, level + 1);
    for (std::vector<uint8_t> child : children)
    {
      _increase_elements_after_level(child, comb_array, n, k, level + 1);
    }
  }
}
}
