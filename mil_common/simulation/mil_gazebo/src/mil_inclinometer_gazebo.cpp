#include <mil_gazebo/mil_inclinometer_gazebo.hpp>
#include <mil_gazebo/mil_gazebo_utils.hpp>

namespace mil_gazebo
{
// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(MilInclinometerGazebo)

MilInclinometerGazebo::MilInclinometerGazebo() : SensorPlugin()
{
}

MilInclinometerGazebo::~MilInclinometerGazebo()
{
}

void MilInclinometerGazebo::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_parent);
  if (!sensor_)
  {
    ROS_ERROR_NAMED("MILInclinometerGazebo", "Parent sensor is null");
    return;
  }

  parent_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(
      gazebo::physics::get_world(sensor_->WorldName())->EntityByName(sensor_->ParentName()));
  if (!parent_)
  {
    ROS_ERROR_NAMED("MILInclinometerGazebo", "Parent link is null");
    return;
  }

  if (_sdf->HasElement("frame_id"))
  {
    frame_name = _sdf->GetElement("frame_id")->Get<std::string>();
  }
  else
  {
    frame_name = sensor_->ParentName();
  }

  if (_sdf->HasElement("topic"))
  {
    topic_name = _sdf->GetElement("topic")->Get<std::string>();
  }

  if (_sdf->HasElement("queue"))
  {
    queue_size = _sdf->GetElement("queue")->Get<int>();
  }

  if (_sdf->HasElement("update_rate"))
  {
    update_rate = _sdf->GetElement("update_rate")->Get<double>();
  }
  
  if(_sdf->HasElement("dynamic_bias_stddev"))
  {
    velocity_random_walk = _sdf->GetElement("dynamic_bias_stddev")->Get<double>();
  }

  if(_sdf->HasElement("dynamic_bias_correlation_time"))
  {
    correlation_time = _sdf->GetElement("dynamic_bias_correlation_time")->Get<double>();
  }

  previous_bias = 0;
  msg_.vel.incremental_linear_velocity_covariance[0] = velocity_random_walk;
  msg_.vel.incremental_linear_velocity_covariance[4] = velocity_random_walk;
  msg_.vel.incremental_linear_velocity_covariance[8] = velocity_random_walk;

  pub_ = nh_.advertise<mil_msgs::IncrementalLinearVelocityStamped>(topic_name, queue_size);

  update_connection_ = sensor_->ConnectUpdated(boost::bind(&MilInclinometerGazebo::OnUpdate, this));
}

void MilInclinometerGazebo::OnUpdate()
{
  if (pub_.getNumSubscribers())
  {
    msg_.header.frame_id = frame_name;
    Convert(sensor_->LastMeasurementTime(), msg_.header.stamp);

    //Calculate incremental velocity
    current_velocity = parent_->RelativeLinearVel();
    incremental_velocity = current_velocity - previous_velocity;
    previous_velocity = current_velocity;

    //Add noise
    double dt = 1 / update_rate;
    incremental_velocity.X(addRandomWalkNoise(incremental_velocity.X(), dt, previous_bias, velocity_random_walk, correlation_time));
    incremental_velocity.Y(addRandomWalkNoise(incremental_velocity.Y(), dt, previous_bias, velocity_random_walk, correlation_time));
    incremental_velocity.Z(addRandomWalkNoise(incremental_velocity.Z(), dt, previous_bias, velocity_random_walk, correlation_time));
    Convert(incremental_velocity, msg_.vel.incremental_linear_velocity);

    pub_.publish(msg_);
  }
}
}  // namespace mil_gazebo