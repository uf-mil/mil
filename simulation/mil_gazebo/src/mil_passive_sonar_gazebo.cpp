#include <mil_msgs/PerceptionObject.h>
#include <mil_gazebo/mil_passive_sonar_gazebo.hpp>
#include <mil_passive_sonar/ProcessedPing.h>

namespace mil_gazebo
{
void MilPassiveSonarGazebo::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  origin_ = _model;

  frame_ = "base_link";
  if (_sdf->HasElement("frame"))
    frame_ = _sdf->GetElement("frame")->Get<std::string>();

  offset_ = gazebo::math::Pose();
  if (_sdf->HasElement("offset"))
    offset_ = _sdf->GetElement("offset")->Get<gazebo::math::Pose>();

  if(_sdf->HasElement("freq"))
    freq_ = _sdf->GetElement("freq")->Get<double>();

  if(_sdf->HasElement("amplitude"))
    amplitude_ = _sdf->GetElement("amplitude")->Get<double>();

  if (!_sdf->HasElement("model"))
  {
    ROS_ERROR_NAMED("MilPassiveSonarGazebo", "missing required attribute <model>");
    return;
  }

  std::string model_name = _sdf->GetElement("model")->Get<std::string>();
  model_ = origin_->GetWorld()->GetModel(model_name);
  if (!model_)
  {
    ROS_ERROR_NAMED("MilPassiveSonarGazebo", "model [%s] does not exist", model_name.c_str());
    return;
  }

  double rate = 1.0;
  if (_sdf->HasElement("rate"))
    rate = _sdf->GetElement("rate")->Get<double>();

  nh_ = ros::NodeHandle("mil_model_heading");
  vector_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/hydrophones/direction", 1);
  processed_ping_pub_ = nh_.advertise<mil_passive_sonar::ProcessedPing>("/hydrophones/processed", 1);
  timer_ = nh_.createTimer(ros::Duration(rate), std::bind(&MilPassiveSonarGazebo::TimerCb, this, std::placeholders::_1));
}

void MilPassiveSonarGazebo::TimerCb(const ros::TimerEvent&)
{
  // Calculate direction to model
  auto world_pose = origin_->GetWorldPose();
  auto origin_pose = world_pose * offset_;
  auto origin_pos = origin_pose.pos;
  auto model_pose = model_->GetWorldPose().pos;
  auto difference = model_pose - origin_pos;
  difference = origin_pose.rot.GetInverse() * difference;
  difference = difference.Normalize();

  // Send direction msg
  geometry_msgs::Vector3Stamped vec;
  GazeboVectorToRosMsg(difference, vec.vector);
  vec.header.frame_id = frame_;
  vec.header.stamp = ros::Time::now();
  vector_pub_.publish(vec);

  // Send processed ping
  mil_passive_sonar::ProcessedPing processed_ping_msg;
  processed_ping_msg.header = vec.header;
  processed_ping_msg.position.x = vec.vector.x;
  processed_ping_msg.position.y = vec.vector.y;
  processed_ping_msg.position.z = vec.vector.z;
  // TODO(kev-the-dev): add noise to freq / amplitude
  processed_ping_msg.freq = freq_;
  processed_ping_msg.amplitude = amplitude_;
  processed_ping_msg.valid = true;
  processed_ping_pub_.publish(processed_ping_msg);
}

void MilPassiveSonarGazebo::GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

GZ_REGISTER_MODEL_PLUGIN(MilPassiveSonarGazebo)
}
