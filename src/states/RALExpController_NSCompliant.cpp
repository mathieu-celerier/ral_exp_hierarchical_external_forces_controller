#include "RALExpController_NSCompliant.h"

#include <RALExpController/RALExpController.h>

void RALExpController_NSCompliant::configure(const mc_rtc::Configuration & config) {}

void RALExpController_NSCompliant::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(5.0);
  ctl.compPostureTask->weight(1);

  ctl.eeTask->reset();
  ctl.eeTask->positionTask->weight(10000);
  ctl.eeTask->positionTask->stiffness(100);
  ctl.eeTask->positionTask->position(Eigen::Vector3d(0.68, 0.0, 0.45));
  ctl.eeTask->orientationTask->weight(10000);
  ctl.eeTask->orientationTask->stiffness(100);
  ctl.eeTask->orientationTask->orientation(Eigen::Quaterniond(-1, 4, 1, 4).normalized().toRotationMatrix());
  ctl.solver().addTask(ctl.eeTask);

  ctl.compPostureTask->makeCompliant(true);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpController_NSCompliant::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  output("OK");
  return true;
}

void RALExpController_NSCompliant::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_NSCompliant", RALExpController_NSCompliant)
