template <class MSG>
mil_tools::TopicPlayer<MSG>::TopicPlayer(ros::NodeHandle* _nh)
{
  nh_ = _nh;

  if (!nh_->getParam("play_topic", topic_))
  {
    ROS_FATAL("topic player could not get play_topic ros param");
    return;
  }
  pub_ = nh_->advertise<MSG>(topic_, 1000);

  enable_service_ = nh_->advertiseService("enable", &TopicPlayer::Enable, this);

  return;
}

template <class MSG>
void mil_tools::TopicPlayer<MSG>::Play(const ros::TimerEvent& event)
{
  auto start = ros::Time::now();
  rosbag::View view(bag_);
  // check type in first msg, make sure is correct
  if (view.begin()->instantiate<MSG>() == nullptr)
  {
    ROS_FATAL("topic player, wrong type in bag file");
    return;
  }
  // get the time of the first message
  ros::Time prev_msg_stamp = view.begin()->instantiate<MSG>()->header.stamp;
  ROS_INFO("Playing");
  for (rosbag::MessageInstance const m : view)
  {
    if (!enabled_)
      break;
    auto msg = m.instantiate<MSG>();
    // if encounter wrong msg types later on, do not publish them and print warn
    if (msg != nullptr)
    {
      auto now = ros::Time::now();
      ros::Duration sleep_for = msg->header.stamp - prev_msg_stamp;
      sleep_for.sleep();
      prev_msg_stamp = msg->header.stamp;
      pub_.publish(*msg);
    }
    else
    {
      ROS_WARN("topic player, wrong messgae type in bag file, skipping");
    }
  }
  bag_.close();
  return;
}

template <class MSG>
bool mil_tools::TopicPlayer<MSG>::Enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  enabled_ = req.data;
  if (enabled_)
  {
    // load file name
    std::string file_name;
    if (!nh_->getParam("file_name", file_name))
    {
      ROS_FATAL("topic player could not get file_name ros param");
      res.success = false;
      return true;
    }

    bag_.open(file_name, rosbag::bagmode::Read);

    res.success = true;
    float start_in = 1.0;
    start_timer_ = nh_->createTimer(ros::Duration(start_in), &TopicPlayer<MSG>::Play, this, true);
  }
  return true;
}
