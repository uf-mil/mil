template<class MSG>
mil_tools::TopicPlayer<MSG>::TopicPlayer(ros::NodeHandle* _nh)
{
    nh_ = _nh;

    // load file name
    std::string file_name;
    if (!nh_->getParam("file_name", file_name))
    {
        ROS_FATAL("topic player could not get file_name ros param");
        return;
    }

    bag_.open(file_name, rosbag::bagmode::Read);

    if (!nh_->getParam("play_topic", topic_))
    {
        ROS_FATAL("topic player could not get play_topic ros param");
        return;
    }
    pub_ = nh_->advertise<MSG>(topic_, 1000);

    enable_service_ = nh_->advertiseService("enable", &TopicPlayer::Enable, this);

    return;
}


template<class MSG>
void mil_tools::TopicPlayer<MSG>::Play(const ros::TimerEvent& event)
{
    std::vector<std::string> topics = {topic_};
    rosbag::View view(bag_, rosbag::TopicQuery(topics));
    auto start = ros::Time::now();
    ros::Duration offset;
    bool first = true;
    for(rosbag::MessageInstance const m : view)
    {
        auto msg = m.instantiate<MSG>();
        if(msg != nullptr)
        {
            if (first)
            {
                offset = msg->header.stamp - start;
                first = false;
            }
            if (msg->header.stamp + offset < ros::Time::now())
                ros::Duration(ros::Time::now() - msg->header.stamp + offset).sleep();
            pub_.publish(*msg);
        }
    }
    return;
}


template<class MSG>
bool mil_tools::TopicPlayer<MSG>::Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    enabled_ = req.data;
    if (enabled_)
    {
        res.success = true;
        float start_in = 1.0;
        //ROS_INFO("playing %d Messages in %f sec", (int)buffer_.size(), start_in);
        start_timer_ = nh_->createTimer(ros::Duration(start_in), &TopicPlayer<MSG>::Play, this, true);
    }
    return true;
}
