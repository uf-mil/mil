#include <mil_msgs/PerceptionObject.h>
#include <mil_gazebo/mil_model_heading.hpp>

namespace mil_gazebo
{
void MILModelHeading::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  origin_ = _model;
  std::string topic = "heading";
  if (_sdf->HasElement("topic"))
    topic = _sdf->GetElement("topic")->Get<std::string>();

  frame_ = "base_link";
  if (_sdf->HasElement("frame"))
    frame_ = _sdf->GetElement("frame")->Get<std::string>();

  offset_ = gazebo::math::Pose();
  if (_sdf->HasElement("offset"))
    offset_ = _sdf->GetElement("offset")->Get<gazebo::math::Pose>();

  if (!_sdf->HasElement("model"))
  {
    ROS_ERROR_NAMED("MILModelHeading", "missing required attribute <model>");
    return;
  }

  std::string model_name = _sdf->GetElement("model")->Get<std::string>();
  model_ = origin_->GetWorld()->GetModel(model_name);
  if (!model_)
  {
    ROS_ERROR_NAMED("MILModelHeading", "model [%s] does not exist", model_name.c_str());
    return;
  }

  double rate = 1.0;
  if (_sdf->HasElement("rate"))
    rate = _sdf->GetElement("rate")->Get<double>();

  nh_ = ros::NodeHandle("mil_model_heading");
  vector_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(topic, 1);
  timer_ = nh_.createTimer(ros::Duration(rate), std::bind(&MILModelHeading::TimerCb, this, std::placeholders::_1));
}

void MILModelHeading::TimerCb(const ros::TimerEvent&)
{
  auto origin_pose = (origin_->GetWorldPose() + offset_).pos;
  auto model_pose = model_->GetWorldPose().pos;
  auto difference = model_pose - origin_pose;
  difference = difference.Normalize();
  geometry_msgs::Vector3Stamped vec;
  GazeboVectorToRosMsg(difference, vec.vector);
  vec.header.frame_id = frame_;
  vec.header.stamp = ros::Time::now();
  vector_pub_.publish(vec);
}

void MILModelHeading::GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

GZ_REGISTER_MODEL_PLUGIN(MILModelHeading)
}
