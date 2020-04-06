
namespace gazebo
{
template <class MSG>
WheelPlugin<MSG>::WheelPlugin()
{
}

template <class MSG>
void WheelPlugin<MSG>::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  GZ_ASSERT(_model != NULL, "Model is NULL");

  model_ = _model;

  GZ_ASSERT(_sdf != NULL, "SDF is NULL");

  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");

  GZ_ASSERT(_sdf->HasElement("wheels"), "No Wheels specified");
  unsigned int i = 0;
  auto wheels = _sdf->GetElement("wheels");
  while(wheels->HasElement("wheel_" + std::to_string(i)))
  {
    wheel_names_.push_back(wheels->Get<std::string>("wheel_" + std::to_string(i)));
    ++i;
  }

  GZ_ASSERT(_sdf->HasElement("topic_name"), "WheelPlugin: no topic_name provided");
  std::string topic_name = _sdf->Get<std::string>("topic_name");
  sub_ = nh_.subscribe(topic_name, 1, &WheelPlugin::Callback, this);
}
}
