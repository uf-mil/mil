/* * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "mil_gazebo/mil_buoyancy_gazebo.hpp"
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

  if (_sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = _sdf->Get<double>("fluid_density");
  }
  if (_sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = _sdf->Get<double>("fluid_level");
  }

  if (!_sdf->HasElement("volume"))
  {
    gzerr << "Required element <volume> missing";
    return;
  }
  this->baseLinkVolume = _sdf->GetElement("volume")->Get<double>();
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
