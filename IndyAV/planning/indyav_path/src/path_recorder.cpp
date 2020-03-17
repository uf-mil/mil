#include <ros/ros.h>

#include <string.h>

#include <indyav_path/path_recorder.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_recorder");
    ros::NodeHandle nh("~");
    std::string env = "";
    if (argc > 1)
        env = std::string(argv[1]);
    else
    {
        ROS_FATAL("environment arg required");
        return 1;
    }

    if (env == "real")
    {
        PathRecorder<ros::WallTime, ros::WallDuration> path_recorder(&nh);
        ros::spin();
    }
    else if (env == "gazebo")
    {
        PathRecorder<ros::Time, ros::Duration> path_recorder(&nh);
        ros::spin();
    }
    else
    {
        ROS_FATAL("environemnt of %s not supported", env.c_str());
        return 1;
    }
    return 0;
}
