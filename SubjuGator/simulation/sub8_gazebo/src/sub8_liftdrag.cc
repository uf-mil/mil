/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "sub8_gazebo/sub8_liftdrag.hpp"

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5 * M_PI;
  this->claStall = 0.0;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(this->linkName);
  }
}

/////////////////////////////////////////////////
void LiftDragPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LiftDragPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  // get linear velocity at cp in inertial frame
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);

  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.Length() <= 0.01)
    return;

  // pose of body
  ignition::math::Pose3d pose = this->link->WorldPose();

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
  ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);

  // ldNormal vector to lift-drag-plane described in inertial frame
  ignition::math::Vector3d ldNormal = forwardI.Cross(upwardI).Normalize();

  // check sweep (angle between vel and lift-drag-plane)
  double sinSweepAngle = ldNormal.Dot(vel) / vel.Length();

  // get cos from trig identity
  double cosSweepAngle2 = (1.0 - sinSweepAngle * sinSweepAngle);
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI : this->sweep + M_PI;

  // angle of attack is the angle between
  // vel projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = ldNormal Xcross ( vector Xcross ldNormal)
  //
  // so,
  // velocity in lift-drag plane (expressed in inertial frame) is:
  ignition::math::Vector3d velInLDPlane = ldNormal.Cross(vel.Cross(ldNormal));

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftDirection = ldNormal.Cross(velInLDPlane);
  liftDirection.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = ldNormal;

  double cosAlpha =
      ignition::math::clamp(forwardI.Dot(velInLDPlane) / (forwardI.Length() * velInLDPlane.Length()), -1.0, 1.0);
  // gzerr << "ca " << forwardI.Dot(velInLDPlane) /
  //   (forwardI.Length() * velInLDPlane.Length()) << "\n";

  // get sign of alpha
  // take upwards component of velocity in lift-drag plane.
  // if sign == upward, then alpha is negative
  double alphaSign = -upwardI.Dot(velInLDPlane) / (upwardI.Length() + velInLDPlane.Length());

  // double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
  if (alphaSign > 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (this->alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall + this->claStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall + this->claStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = this->cla * this->alpha * cosSweepAngle2;

  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q * this->area * liftDirection;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (this->alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall + this->cdaStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
  }
  else if (this->alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall + this->cdaStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
  }
  else
    cd = (this->cda * this->alpha) * cosSweepAngle2;

  // make sure drag is positive
  cd = fabs(cd);

  // drag at cp
  ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall + this->cmaStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall + this->cmaStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * this->alpha * cosSweepAngle2;

  // reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

  // moment arm from cg to cp in inertial plane
  ignition::math::Vector3d momentArm = pose.Rot().RotateVector(this->cp - this->link->GetInertial()->CoG());
  // gzerr << this->cp << " : " << this->link->GetInertial()->CoG() << "\n";

  // force and torque about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;
  // + moment.Cross(momentArm);

  ignition::math::Vector3d torque = moment;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (0)
  {
    gzerr << "=============================\n";
    gzerr << "Link: [" << this->link->GetName() << "] pose: [" << pose << "] dynamic pressure: [" << q << "]\n";
    gzerr << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
    gzerr << "spd sweep: [" << velInLDPlane.Length() << "] vel in LD: [" << velInLDPlane << "]\n";
    gzerr << "forward (inertial): " << forwardI << "\n";
    gzerr << "upward (inertial): " << upwardI << "\n";
    gzerr << "lift dir (inertial): " << liftDirection << "\n";
    gzerr << "LD Normal: " << ldNormal << "\n";
    gzerr << "sweep: " << this->sweep << "\n";
    gzerr << "alpha: " << this->alpha << "\n";
    gzerr << "lift: " << lift << "\n";
    gzerr << "drag: " << drag << " cd: " << cd << " cda: " << this->cda << "\n";
    gzerr << "moment: " << moment << "\n";
    gzerr << "cp momentArm: " << momentArm << "\n";
    gzerr << "force: " << force << "\n";
    gzerr << "torque: " << torque << "\n";
  }

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(torque);
}
