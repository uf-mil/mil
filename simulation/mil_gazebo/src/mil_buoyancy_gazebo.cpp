#include <mil_gazebo/mil_buoyancy_gazebo.hpp>
#include <mil_gazebo/mil_gazebo_utils.hpp>
#include <functional>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <string>

#include <ros/ros.h>

using namespace gazebo;

/////////////////////////////////////////////////
MILBuoyancyGazebo::MILBuoyancyGazebo() : fluidDensity(999.1026), fluidLevel(0.0)
{
}

/////////////////////////////////////////////////
void MILBuoyancyGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gravity = _model->GetWorld()->GetPhysicsEngine()->GetGravity();

  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");

  this->baseLink = _model->GetLink();
  GZ_ASSERT(this->baseLink != NULL, "Base link is null");

  mil_gazebo::GetFromSDFOrRosParam(_sdf, "fluid_density", fluidDensity);
  mil_gazebo::GetFromSDFOrRosParam(_sdf, "fluid_level", fluidLevel);
  if (!mil_gazebo::GetFromSDFOrRosParam(_sdf, "volume", baseLinkVolume)) {
    ROS_ERROR("volume param not set");
  }
}

/////////////////////////////////////////////////
void MILBuoyancyGazebo::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MILBuoyancyGazebo::OnUpdate, this));
}

/////////////////////////////////////////////////
void MILBuoyancyGazebo::OnUpdate()
{
  double height = baseLink->GetBoundingBox().GetSize().z;
  double bottomRelSurf = this->fluidLevel - (baseLink->GetWorldPose().pos.z - height / 2.0);
  const math::Vector3 kGravity(0, 0, -9.81);

  double volume = baseLinkVolume;
  // out of water
  if (bottomRelSurf <= 0)
  {
    volume = 0.0;
  }
  // at surface
  else if (bottomRelSurf <= height)
  {
    volume = (bottomRelSurf / height) * volume;
  }
  math::Vector3 buoyancy = -this->fluidDensity * volume * gravity;

  baseLink->AddForce(buoyancy);
}

GZ_REGISTER_MODEL_PLUGIN(MILBuoyancyGazebo)
