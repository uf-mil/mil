#include "navigator_gazebo/navigator_thrusters.hpp"
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

ThrusterPlugin::ThrusterPlugin()
{
}

void ThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");

  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  // Make sure the ROS node for Gazebo has already been initialized
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");
  blSub = nh.subscribe("BL_motor/cmd", 1, &ThrusterPlugin::BLCallback, this);
  brSub = nh.subscribe("BR_motor/cmd", 1, &ThrusterPlugin::BRCallback, this);
  flSub = nh.subscribe("FL_motor/cmd", 1, &ThrusterPlugin::FLCallback, this);
  frSub = nh.subscribe("FR_motor/cmd", 1, &ThrusterPlugin::FRCallback, this);

  // Load Thruster Configuration
  this->thrusters.push_back(Thruster("BL"));
  this->thrusters.push_back(Thruster("BR"));
  this->thrusters.push_back(Thruster("FL"));
  this->thrusters.push_back(Thruster("FR"));

  // Apply to multiple links
  GZ_ASSERT(this->sdf->HasElement("link"), "Must have a link specified!");

  std::string link_name = this->sdf->Get<std::string>("link");
  this->targetLink = this->model->GetLink(link_name);
  GZ_ASSERT(this->targetLink, "Could not find specified link");
}

void ThrusterPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::OnUpdate, this));
}

// Callbacks for each thrusters' command
void ThrusterPlugin::BLCallback(const roboteq_msgs::Command::ConstPtr &command)
{
  mtx.lock();
  this->lastTime = ros::Time::now();
  this->commands[0] = command->setpoint;
  mtx.unlock();
}

void ThrusterPlugin::BRCallback(const roboteq_msgs::Command::ConstPtr &command)
{
  mtx.lock();
  this->lastTime = ros::Time::now();
  this->commands[1] = command->setpoint;
  mtx.unlock();
}

void ThrusterPlugin::FLCallback(const roboteq_msgs::Command::ConstPtr &command)
{
  mtx.lock();
  this->lastTime = ros::Time::now();
  this->commands[2] = command->setpoint;
  mtx.unlock();
}

void ThrusterPlugin::FRCallback(const roboteq_msgs::Command::ConstPtr &command)
{
  mtx.lock();
  this->lastTime = ros::Time::now();
  this->commands[3] = command->setpoint;
  mtx.unlock();
}

void ThrusterPlugin::OnUpdate()
{
  if ((ros::Time::now() - this->lastTime) > ros::Duration(2.0))
  {
    return;
  }

  mtx.lock();
  std::vector<double> cmdBuffer;
  cmdBuffer = this->commands;
  mtx.unlock();

  GZ_ASSERT(this->targetLink, "Could not find specified link");

  math::Vector3 net_force(0.0, 0.0, 0.0);
  math::Vector3 net_torque(0.0, 0.0, 0.0);

  for (uint i = 0; i < cmdBuffer.size(); i++)
  {
    // Clamp thrust value, convert to newton, then accumulate for net force

    double abs_thrust = std::abs(cmdBuffer[i]);
    double sign = cmdBuffer[i] / abs_thrust;
    if (abs_thrust > this->thrusters[i].effort_limit)
    {
      // Out of range
      cmdBuffer[i] = sign * this->thrusters[i].effort_limit;
    }

    double newton_thrust = cmdBuffer[i] / this->thrusters[i].effort_ratio;
    math::Vector3 force = this->thrusters[i].direction * newton_thrust;
    math::Vector3 torque = this->thrusters[i].position.Cross(force);
    net_force += force;
    net_torque += torque;
  }

  /*  ROS_INFO_THROTTLE(1, "things: %f, %f, %f, %f", cmdBuffer[0], cmdBuffer[1], cmdBuffer[2], cmdBuffer[3]);
    ROS_INFO_THROTTLE(1, "force: %f, %f, %f", force[0], force[1], force[2]);
    ROS_INFO_THROTTLE(1, "torque: %f, %f, %f", net_torque[0], net_torque[1], net_torque[2]);
  */

  this->targetLink->AddRelativeForce(net_force);
  this->targetLink->AddRelativeTorque(net_torque);
}
}
