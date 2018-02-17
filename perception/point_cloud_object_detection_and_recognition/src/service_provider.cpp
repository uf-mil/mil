#include <point_cloud_object_detection_and_recognition/service_provider.hpp>

// #include <ros/console.h>

namespace pcodar
{

void service_provider::initialize(ros::NodeHandle& nh)
{
    modify_classification_service_ = nh.advertiseService("/database/change_classification", &pcodar::service_provider::DBQuery_cb, this);
}

 void service_provider::update_objects_reference(id_object_map_ptr objects)
{ 
    objects_ = objects;
}

bool service_provider::DBQuery_cb(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response &res)
{
    if(req.cmd != "")
    {
        int pos = req.cmd.find_first_of("=");
        
        int id = -1;
        try {
            id = std::stoi(req.cmd.substr(0, pos));
        }
        catch(...)
        {
            res.found = false;
            return false;
        }
        std::string cmd = req.cmd.substr(pos+1);
        std::cout << id << " " << cmd << std::endl;
        auto it = objects_->find(id);
        if (it != objects_->end())
        {
            it->second.labeled_classification = cmd;
        }
        else {
            std::cout << "Failed to find" << std::endl;
            return false;
        }
    }
    return true;
}


}  // pcodar namespace
