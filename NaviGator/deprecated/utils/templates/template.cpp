
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida. 
It is writen for use on the UF ___________ robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: This is your title
Start Date: Date

Author: 
Author email: 

Co-author:
Co-author email:

CODE DETAILS --------------------------------------------------------------------

Please include inputs, outputs, and fill with a pseudo-code or description of the source to follow

inputs: /topic
output: /topic

1. Step 1
2. Step 2
3. Step 3
N. Step N

'''

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <cmath>
// Need to create a C++ compatible kill handling section

class CLASS_NAME
{

private:

	bool killed;

	void state_status()
	{
	}

	void set_kill(const msg_folder::msg_type_set_kill::ConstPtr& name)
	{
		killed = true;
	}

	void clear_kill(const msg_folder::msg_type_clear_kill::ConstPtr& name)
	{
		killed = false;
	}

public:

	ros::NodeHandle nh;
	std::string kill_handling = nh.resolveName("KILL_HANDLING");
	killed = false;
	laser_sub_ = nh.subscribe<msg_folder::msg_type_set_kill>(kill_handling.c_str(), 10, &CLASS::set_kill, this);
	laser_sub_ = nh.subscribe<msg_folder::msg_type_clear_kill>(kill_handling.c_str(), 10, &CLASS::clear_kill, this);


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "NODE_NAME");
	ros::NodeHandle nh;
	ros::Rate update_rate(100);	

	CLASS_NAME class_instance_variable;

	while(ros::ok())
	{
		class_instance_variable.state_status()
		update_rate.sleep();
		// Calls callbacks in queue
		ros::spinOnce();
	}

	return 0;
}

