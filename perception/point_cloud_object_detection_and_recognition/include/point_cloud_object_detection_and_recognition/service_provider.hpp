#pragma once

#include "pcodar_types.hpp"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>


#include <visualization_msgs/MarkerArray.h>

#include <memory>

#include <mil_msgs/ObjectDBQuery.h>


namespace pcodar
{
class service_provider
{
   public:
    void initialize(ros::NodeHandle& nh, id_label_map_ptr id_label_map);
    // TODO: Since there are now smart pointers, just move them to initalize instead of reseting theme
    void update_objects_reference(mil_msgs::PerceptionObjectArrayPtr objects);

   private:
    ros::ServiceServer modify_classification_service_;
    mil_msgs::PerceptionObjectArrayPtr objects_;
    id_label_map_ptr id_label_map_;

    bool DBQuery_cb(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response &res);

};
}