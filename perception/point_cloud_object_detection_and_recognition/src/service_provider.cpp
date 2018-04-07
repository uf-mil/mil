#include <point_cloud_object_detection_and_recognition/service_provider.hpp>

// #include <ros/console.h>

namespace pcodar {
std::unique_ptr<dynamic_reconfigure::Client<navigator_tools::BoundsConfig>>
    bounds_client;

bool bounds_update_cb(const navigator_tools::BoundsConfig &config) {
  std::cout << "Updated bounds " << std::endl;
  boundary[0](0) = config.x1;
  boundary[0](1) = config.y1;
  boundary[1](0) = config.x2;
  boundary[1](1) = config.y2;
  boundary[2](0) = config.x3;
  boundary[2](1) = config.y3;
  boundary[3](0) = config.x4;
  boundary[3](1) = config.y4;
}

void service_provider::initialize(ros::NodeHandle &nh,
                                  id_object_map_ptr id_object_map) {
  modify_classification_service_ = nh.advertiseService(
      "/database/requests", &pcodar::service_provider::DBQuery_cb, this);
  id_object_map_ = id_object_map;
  bounds_client.reset(
      new dynamic_reconfigure::Client<navigator_tools::BoundsConfig>(
          "/bounds_server", nh, &bounds_update_cb));
}

void service_provider::update_objects_reference(
    mil_msgs::PerceptionObjectArrayPtr objects) {
  objects_ = objects;
}

bool service_provider::DBQuery_cb(mil_msgs::ObjectDBQuery::Request &req,
                                  mil_msgs::ObjectDBQuery::Response &res) {
  if (req.cmd != "") {
    int pos = req.cmd.find_first_of("=");

    int id = -1;
    try {
      id = std::stoi(req.cmd.substr(0, pos));
    } catch (...) {
      std::cout << "Could not find id " << id << std::endl;
      res.found = false;
      return false;
    }
    std::string cmd = req.cmd.substr(pos + 1);
    auto it = id_object_map_->find(id);
    if (it == id_object_map_->end()) {
      std::cout << "Could not find id " << id << std::endl;
      return false;
    } else {
      std::cout << "set " << id << " to " << cmd << std::endl;
      it->second.labeled_classification = cmd;
    }
  }
  std::vector<mil_msgs::PerceptionObject> objects(id_object_map_->size());
  int i = 0;
  for (auto &o : *id_object_map_) {
    objects[i] = o.second;
    i++;
  }
  if (req.name == "all") {
    res.found = true;
    res.objects = objects;
    return true;
  }

  // if (std::find(classification_strings.begin(), classification_strings.end(),
                // req.name) != classification_strings.end()) {
  if (req.name != "")
    for (const auto &object : objects) {
      if (object.classification == req.name ||
          object.labeled_classification == req.name) {
        res.found = true;
        res.objects.push_back(object);
      }
    }
  // }
  return true;
}

}  // pcodar namespace
