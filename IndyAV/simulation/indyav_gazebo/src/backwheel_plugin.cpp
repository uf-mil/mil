#include <indyav_gazebo/backwheel_plugin.hpp>
#include <iostream>

namespace gazebo
{
BackWheelPlugin::BackWheelPlugin() : WheelPlugin()
{
}

void BackWheelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  WheelPlugin::Load(_model, _sdf);

  GZ_ASSERT(_sdf->HasElement("max_velocity"), "no max velocity provided");
  max_velocity_ = _sdf->Get<double>("max_velocity");

  GZ_ASSERT(_sdf->HasElement("wheel_radius"), "no wheel_radius provided");
  wheel_radius_ = _sdf->Get<double>("wheel_radius");

  GZ_ASSERT(_sdf->HasElement("engine_to_wheel_ratio"), "no engine_to_wheel_ratio provided");
  engine_to_wheel_ratio_ = _sdf->Get<double>("engine_to_wheel_ratio");

  GZ_ASSERT(_sdf->HasElement("back_axle_joint"), "no back axle provided");
  back_axle_joint_name_ = _sdf->Get<std::string>("back_axle_joint");
  back_axle_joint_ = model_->GetJoint(back_axle_joint_name_);
}

void BackWheelPlugin::Callback(const indyav_control::ThrottleBrakeStamped& _msg)
{
    cmd_ = _msg.cmd;
}

void BackWheelPlugin::OnUpdate()
{
  // approx the FR 125 max rpm to kW with auto clutch mapping
  auto rpm_to_kw = [](const double& rpm) -> double
  {
    if(rpm < 3000)
      return 3;
    else if (rpm >= 3000 && rpm < 12000)
      return (18.0/9000.0)*rpm - 3;
    else if (rpm >= 12000 && rpm < 14500)
      return (-16.0/2500.0)*rpm + 97.8;
    else
      return 0;
  };

  double vel = model_->RelativeLinearVel().X();
  std::cout << "vel: " << vel << "\n";
  if (vel < max_velocity_)
  {
    if (vel < 0.01)
      vel = 0.01;
    double rpm = ((vel / (2*M_PI*wheel_radius_)) * 60) * engine_to_wheel_ratio_;
    double kw = rpm_to_kw(rpm) * cmd_ * 100;
    // not sure why, but this is nessesary to make it resemble real life
    // reference: http://wentec.com/unipower/calculators/power_torque.asp
    double t = (kw * 9.5488) / rpm;
    back_axle_joint_->SetForce(0, t);
    std::cout << "rpm: " << rpm << "\n";
    std::cout << "torque: " << t << "\n";
  }
  else
  {
    back_axle_joint_->SetForce(0, 0);
  }
}

void BackWheelPlugin::Init()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BackWheelPlugin::OnUpdate, this));
}
}
