/**
* Author: Patrick Emami
* Date: 10/23/15
*/

#include "tgen_thruster_info.h"

using sub8::trajectory_generator::TGenThrusterInfo;

TGenThrusterInfo::TGenThrusterInfo() {
  // Load the thruster lever arms and directions matrices
  XmlRpc::XmlRpcValue XmlBusses; 

  if (ros::param::get("/busses", XmlBusses)) {
  	// TODO
  }

}