#pragma once

#include "pcodar_types.hpp"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <mil_msgs/PerceptionObject.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace pcodar
{
class marker_manager
{
   public:
    void initialize(ros::NodeHandle& nh);

    void update_markers(const std::vector<mil_msgs::PerceptionObject>& objects);
    
   private:
    ros::Publisher pub_markers_;
};
}
