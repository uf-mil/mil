#pragma once

#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>

namespace subjugator_gazebo
{
/*
  Struct used for encoding the comms layout of the thrusters on the sub
*/
struct ThrusterPort
{
  std::string port_name;
  std::vector<std::string> thruster_names;
};

/*
  Thruster Definition
*/
struct ThrusterDef
{
  int motor_id;
  double position[3];
  double direction[3];
  double bounds[2];
};

/*
  Loads portst definitions from an XmlRpcValue
  params:
  - ports_xmlrpc: list of dicts with structure [(port, [thruster_name0, ...]), ...)
  - ports_vec: reference to vector in which to store the loaded ThrusterPort's
*/
void load_ports(XmlRpc::XmlRpcValue& ports_xmlrpc, std::vector<ThrusterPort>& ports_vec);

/*
  Loads portst definitions from an XmlRpcValue
  params:
  - ports_xmlrpc: dict with structure ((motor_id, position, direction, bounds), ...)
  - ports_vec: reference to map in which to store the loaded ThrusterDef's
*/
void load_thrusters(XmlRpc::XmlRpcValue& thrusters_xmlrpc, std::map<std::string, ThrusterDef>& thruster_map);

}  // namespace subjugator_gazebo
