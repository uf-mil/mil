#pragma once

#include <map>
#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  class ThrusterPlugin : public ModelPlugin
  {
    public: ThrusterPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    protected: virtual void OnUpdate();

    protected: event::ConnectionPtr updateConnection;

    protected: physics::ModelPtr model;

    protected: physics::PhysicsEnginePtr physicsEngine;

    protected: sdf::ElementPtr sdf;

    // protected: double fluidDensity;
    // protected: double dragCoeff;
    // protected: std::map<int, VolumeProperties> volPropsMap;
  };
}

