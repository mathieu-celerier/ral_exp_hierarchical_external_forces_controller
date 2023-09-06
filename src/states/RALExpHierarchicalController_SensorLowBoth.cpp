#include "RALExpHierarchicalController_SensorLowBoth.h"

#include <RALExpHierarchicalController/RALExpHierarchicalController.h>

void RALExpHierarchicalController_SensorLowBoth::configure(const mc_rtc::Configuration & config) {}

void RALExpHierarchicalController_SensorLowBoth::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);

  // Deactivate feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", LOW_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(20.0);

  ctl.waitingForInput = true;

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpHierarchicalController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpHierarchicalController_SensorLowBoth::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);

  if(not ctl.waitingForInput)
  {
    output("OK");
    return true;
  }

  return false;
}

void RALExpHierarchicalController_SensorLowBoth::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpHierarchicalController_SensorLowBoth", RALExpHierarchicalController_SensorLowBoth)
