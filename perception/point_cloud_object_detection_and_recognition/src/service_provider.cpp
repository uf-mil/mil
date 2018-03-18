#include <point_cloud_object_detection_and_recognition/service_provider.hpp>

// #include <ros/console.h>

namespace pcodar
{

void service_provider::initialize(ros::NodeHandle& nh, id_label_map_ptr id_label_map)
{
    modify_classification_service_ = nh.advertiseService("/database/requests", &pcodar::service_provider::DBQuery_cb, this);
    id_label_map_ = id_label_map;
  // ros::NodeHandle bounds_nh("/bounds_server");
    // bounds_client_.reset(new dynamic_reconfigure::Client<navigator_tools::BoundsConfig>("/bounds_server", bounds_nh, &pcodar::service_provider::bounds_update_cb));
}

 void service_provider::update_objects_reference(mil_msgs::PerceptionObjectArrayPtr objects)
{ 
    objects_ = objects;
}

bool service_provider::bounds_update_cb(const navigator_tools::BoundsConfig &config)
{
  boundary[0](0) = config.x1;
  boundary[0](1) = config.y1;
  boundary[1](0) = config.x2;
  boundary[1](1) = config.y2;
  boundary[2](0) = config.x3;
  boundary[2](1) = config.y3;
  boundary[3](0) = config.x4;
  boundary[3](1) = config.y4;
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

        for (auto &object : objects_->objects)
        {
            if (object.id == id)
            {
                object.labeled_classification = cmd;
                auto it = id_label_map_->find(id);
                if (it == id_label_map_->end())
                    id_label_map_->insert({id, std::make_pair(object.classification,cmd)});
                else
                    it->second = std::make_pair(object.classification, cmd);
            }        
        }
    }
    if (std::find(classification_strings.begin(), classification_strings.end(), req.name) != classification_strings.end())
    {

        if (req.name == "all")
        {
            res.found = true;
            res.objects = objects_->objects;
            return true;
        }

        for (const auto& object : objects_->objects)
        {
            if (object.classification == req.name || object.labeled_classification == req.name)
            {
                res.found = true;
                res.objects.push_back(object);
            }
        }
    }
    return true;
}


}  // pcodar namespace
