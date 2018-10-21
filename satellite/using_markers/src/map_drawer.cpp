#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "map_drawer");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud>("map_drawer", 1);

  while (ros::ok())
  {
    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "/gps";
    
  }
}
