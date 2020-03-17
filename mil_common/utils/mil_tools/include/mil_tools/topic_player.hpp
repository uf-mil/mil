#pragma once

#include <string.h>
#include <stdio.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace mil_tools
{
template<class MSG>
class TopicPlayer
{
    public:
        TopicPlayer(ros::NodeHandle* _nh);
        virtual bool Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    protected:
        ros::NodeHandle* nh_;

        std::string topic_;

        //std::vector<MSG> buffer_;

        rosbag::Bag bag_;

        bool enabled_ = false;
        ros::ServiceServer enable_service_;

        ros::Timer start_timer_;
        virtual void Play(const ros::TimerEvent& event);
        ros::Publisher pub_;
};
}

#include "../../src/mil_tools/topic_player.cpp"

