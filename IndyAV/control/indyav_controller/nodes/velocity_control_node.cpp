#include "ros/ros.h"
#include "indyav_controller/PIDController.hpp"
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>

struct VelocityListener
{
	std::string name;
	double x_velocity;
	double y_velocity;
	void callback(const nav_msgs::Odometry::ConstPtr& data);
};

void VelocityListener::callback(const nav_msgs::Odometry::ConstPtr& data)
{
	this->x_velocity = data->twist.twist.linear.x;
	this->y_velocity = data->twist.twist.linear.y;
	ROS_INFO("[%s] Linear Velocity -> x: [%f], y: [%f]", 
		this->name, this->x_velocity, this->y_velocity);
}

double vector_magnitude(double &x, double &y)
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

int main(int argc, char **argv)
{
	//Initilize Node and NodeHandle
	ros::init(argc, argv, "velocity_control_node");
    ros::NodeHandle nh;

	//////////  Retrieving ROS Params //////////
	
	//Define PID values
	double kp, ki, kd, min, max, cycle_time;
	nh.getParam("pid_tuning/kp", kp);
	nh.getParam("pid_tuning/ki", ki);
	nh.getParam("pid_tuning/kd", kd);
	nh.getParam("pid_tuning/cycle_time", cycle_time); //Loop time (Hz)
	
	//Define car attributes
	double vehicle_mass, surface_area;
	nh.getParam("car_attributes/mass", vehicle_mass);
	nh.getParam("car_attributes/surface_area", surface_area); 

	//Define drag constants
	double air_density, drag_coeff;
	nh.getParam("drag/air_density", air_density);
	nh.getParam("drag/drag_coefficient", drag_coeff); 
	
	//Retrieve waypoint topic name
	std::string waypoint_info;
	nh.getParam("topics/waypoints", waypoint_info);

	////////////////////////////////////////////
	
	//Create listener objects for waypoint and odom callbacks
	VelocityListener waypoint_listener, odom_listener;
	waypoint_listener.name = "Waypoint";
	odom_listener.name = "Odom";

	//Create ROS publisher/subscribers
    ros::Publisher output_pub = nh.advertise<std_msgs::Float64>("/velocity_control", 10);
	ros::Subscriber waypoint_sub = nh.subscribe<nav_msgs::Odometry>(waypoint_info, 10, &VelocityListener::callback, &waypoint_listener);
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &VelocityListener::callback, &odom_listener);

	//Initialize PID Controller, other variables
	PID controller = PID(kp, ki, kd, min, max, cycle_time);
	double current_speed, target_speed, desired_speed, force, drag;
	std_msgs::Float64 output;

	ros::Rate loop_rate(cycle_time);
    while (ros::ok())
    {
		//Retrieve current speed from odom and target speed from waypoint
		current_speed = vector_magnitude(odom_listener.x_velocity, odom_listener.y_velocity);
		target_speed = vector_magnitude(waypoint_listener.x_velocity, waypoint_listener.y_velocity);

		//Retrieve desired_speed from PID, then calculate necessary Force (N) to reach that speed
	    desired_speed = controller.calculate(target_speed, current_speed);
		force = ((desired_speed - current_speed)/(cycle_time))*vehicle_mass;
		
		//Find drag at current speed and add as a force forwards
		drag = 0.5 * air_density * pow(current_speed, 2) * drag_coeff * surface_area;
		force += drag;

		//Publish Output
		output.data = force;
		output_pub.publish(output);
		ros::spinOnce();
    }
	return 0;
}
