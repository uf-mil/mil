#include <mil_tools/mil_tools.hpp>

namespace mil{
namespace tools{

using namespace std;

vector<string> getRectifiedImageTopics(bool color)
{
  // get all currently published topics from master
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  // Define lambda to determine if a topic is a rectified img topic
  string target_pattern;
  target_pattern = color? "image_rect_color" : "image_rect";
  auto isRectImgTopic = [&target_pattern](ros::master::TopicInfo topic_info)
    {
      // Find last slash
      size_t final_slash = topic_info.name.rfind('/');

      // Match end of topic name to image_rect pattern
      if(final_slash == string::npos)
        return false;
      else
      {
        string topic_name_end = topic_info.name.substr(final_slash + 1);
        return (topic_name_end.find(target_pattern) != string::npos)?  true : false;
      }
    }; // end lambda isRectImgTopic

    // return list of rectified image topics
    vector<string> image_rect_topics;
    for(ros::master::TopicInfo topic : master_topics)
    {
      if(isRectImgTopic(topic))
        image_rect_topics.push_back(topic.name);
    }
    return image_rect_topics;
}


void sortContours(vector<vector<cv::Point2i>>& contour_vec, bool ascending)
{
  auto comp_length =  ascending?
    [](vector<cv::Point2i> const &contour1, vector<cv::Point2i> const &contour2)
      { return  fabs(arcLength(contour1, true)) < fabs(arcLength(contour2, true)); } :
    [](vector<cv::Point2i> const &contour1, vector<cv::Point2i> const &contour2)
      { return  fabs(arcLength(contour1, true)) > fabs(arcLength(contour2, true)); };

  sort(contour_vec.begin(), contour_vec.end(), comp_length);
}


}  // namespace mil::tools  
}  // namespace mil

