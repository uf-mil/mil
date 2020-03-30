template<class MSG>
mil_tools::TopicRecorder<MSG>::TopicRecorder(ros::NodeHandle* _nh)
{
    nh_ = _nh;

    if (!nh_->getParam("file_name", file_name_))
    {
        ROS_FATAL("topic recorder file_name ros param not found");
        return;
    }

    bag_.open(file_name_, rosbag::bagmode::Write);

    if (!nh_->getParam("record_topic", topic_))
    {
        ROS_FATAL("topic recorder record_topic ros param not found");
        return;
    }
    sub_ = nh_->subscribe(topic_, 100, &TopicRecorder::CallBack, this);
    enable_service_ = nh_->advertiseService("enable", &TopicRecorder<MSG>::Enable, this);
}


template<class MSG>
void mil_tools::TopicRecorder<MSG>::CallBack(const MSG& _msg)
{
    if (!enabled_)
        return;
    ++message_count_;
    bag_.write(topic_, ros::Time::now(), _msg);
}


template<class MSG>
bool mil_tools::TopicRecorder<MSG>::Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    enabled_ = req.data;
    if (!enabled_)
    {
        ROS_INFO("%d messages written to %s", message_count_, file_name_.c_str());
        bag_.close();
    }
    if (!nh_->getParam("file_name", file_name_))
    {
        ROS_FATAL("topic recorder file_name ros param not found");
        res.success = false;
        return false;
    }
    res.success = true;
    return true;
}
