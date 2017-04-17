#include <sub8_gazebo/sub8_thruster_config.hpp>

using XmlRpc::XmlRpcValue;
using XmlRpc::XmlRpcException;
using namespace std;

namespace sub8_gazebo
{

void load_ports(XmlRpcValue& ports_xmlrpc, vector<ThrusterPort>& ports_vec)
try
{
  ports_vec = vector<ThrusterPort>();  // Reset vector

  if(!(ports_xmlrpc.getType() == XmlRpcValue::Type::TypeArray))
  {
    auto err_str = "Error loading thruster defintions: param needs to be a dictionary";
    ROS_ERROR("%s", err_str);
    return;
  }

  for(int i = 0; i < ports_xmlrpc.size(); ++i)  // Iterate through list of port defs
  {
    auto port_def = ports_xmlrpc[i];
    auto port = ThrusterPort();
    auto names = port_def["thruster_names"];

    port.port_name = (string)port_def["port"];
    for(int j = 0; j < names.size(); ++j)
      port.thruster_names.push_back(names[j]);

    ports_vec.push_back(port);
  }
}
catch(XmlRpcException& e)
{
  auto err_str = "Error parsing port definitions: %s";
  ROS_ERROR(err_str, e.getMessage());
  return;
}

void load_thrusters(XmlRpcValue& thrusters_xmlrpc, map<string, ThrusterDef>& thruster_map)
try
{
  thruster_map = map<string, ThrusterDef>();  // Reset map

  if(!(thrusters_xmlrpc.getType() == XmlRpcValue::Type::TypeStruct))
  {
    auto err_str = "Error loading thruster defintions: param needs to be a dictionary";
    ROS_ERROR("%s", err_str);
    return;
  }

  for(auto& thruster : thrusters_xmlrpc)  // Iterater through thruster definitions (XmlRpcValues)
  {
    bool is_valid = thruster.second.hasMember("motor_id") &&
                    thruster.second.hasMember("position") &&
                    thruster.second.hasMember("direction") &&
                    thruster.second.hasMember("bounds");
    if(!is_valid)
    {
      auto err_str = "Object %s is not a valid thruster definition. It needs fields"
        " (motor_id, position, direction, bounds)";
      ROS_ERROR(err_str, string(thruster.first).c_str());
      continue;
    }

    ThrusterDef t;
    try
    {
      auto props = thruster.second;
      t.motor_id = props["motor_id"];
      t.position[0] = props["position"][0];
      t.position[1] = props["position"][1];
      t.position[2] = props["position"][2];
      t.direction[0] = props["direction"][0];
      t.direction[1] = props["direction"][1];
      t.direction[2] = props["direction"][2];
      t.bounds[0] = props["bounds"][0];
      t.bounds[1] = props["bounds"][1];

      thruster_map[thruster.first] = ThrusterDef(t);
    }
    catch(XmlRpcException& e)
    {
      auto err_str = "Error parsing thruster definition (%s): %s";
      ROS_ERROR(err_str, thruster.first, e.getMessage());
      continue;
    }
  }
}
catch(XmlRpcException& e)
{
    auto err_str = "Error loading thruster definitions: %s";
    ROS_ERROR(err_str, e.getMessage());
    return;
}

}  // namespace sub8_gazebo

