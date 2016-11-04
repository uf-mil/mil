#include <ros/ros.h>
#include "CameraLidarTransformer.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "camera_lidar_transformer");
    CameraLidarTransformer transformer;
    ros::spin ();
}
