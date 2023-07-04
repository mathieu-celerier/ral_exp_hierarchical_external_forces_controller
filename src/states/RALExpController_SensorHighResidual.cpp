#include "RALExpController_SensorHighResidual.h"

#include <RALExpController/RALExpController.h>

void RALExpController_SensorHighResidual::configure(const mc_rtc::Configuration & config) {}

void RALExpController_SensorHighResidual::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Disable force sensor usage if active
  if(ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(20.0);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_SensorHighResidual::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  output("OK");
  return true;
}

void RALExpController_SensorHighResidual::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_SensorHighResidual", RALExpController_SensorHighResidual)
