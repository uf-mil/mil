/**
* Author: Patrick Emami
* Date: 9/21/15
*/
#include <ros/ros.h>
#include "sub8_state_space.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_generator"); 

	// TODO add Pub/Sub/Srv things 

	while(ros::ok()) {
		ros::spin(); 
	}
}