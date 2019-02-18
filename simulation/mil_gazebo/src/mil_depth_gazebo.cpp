#include <mil_gazebo/mil_depth_gazebo.hpp>

namespace mil_gazebo
{
// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(MilDepthGazebo)

MilDepthGazebo::MilDepthGazebo()
{
}

MilDepthGazebo::~MilDepthGazebo()
{
}

void MilDepthGazebo::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  if (!model_)
  {
    ROS_ERROR_NAMED("MilDepthGazebo", "Parent sensor is null");
    return;
  }

  if (_sdf->HasElement("frame_id"))
  {
    frame_name_ = _sdf->GetElement("frame_id")->Get<std::string>();
  }
  else
    frame_name_ = model_->GetName();

  if (_sdf->HasElement("offset"))
  {
    offset_ = _sdf->GetElement("offset")->Get<ignition::math::Pose3d>();
  }

  if (_sdf->HasElement("update_rate"))
  {
    update_period_ = 1.0 / _sdf->GetElement("update_rate")->Get<double>();
  }

  depth_pub_ = nh_.advertise<mil_msgs::DepthStamped>("/depth", 20);

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MilDepthGazebo::OnUpdate, this, std::placeholders::_1));
}

void MilDepthGazebo::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  auto time = info.simTime;
  if (info.simTime < last_pub_time_) {
    last_pub_time_ = info.simTime;
    return;
  }

  if ( (time - last_pub_time_) < update_period_) {
    return;
  }

  last_pub_time_ = time;

  // TODO: limit publish frequency
  auto pose = model_->GetWorldPose() + offset_;
  double depth = -pose.pos.z;
  if (depth < 0.) depth = 0.;

  mil_msgs::DepthStamped msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp.sec = time.sec;
  msg.header.stamp.nsec = time.nsec;
  msg.depth = depth;
  depth_pub_.publish(msg);
}

}
