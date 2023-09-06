#include "RALExpHierarchicalController_VelLimitPose.h"

#include <RALExpHierarchicalController/RALExpHierarchicalController.h>

void RALExpHierarchicalController_VelLimitPose::configure(const mc_rtc::Configuration & config) {}

void RALExpHierarchicalController_VelLimitPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.5, false, true));
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);

  // Deactivate feedback from external forces estimator (safer)
  if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.2);
  ctl.compPostureTask->target(ctl.postureVelLimit);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.eeTask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RALExpHierarchicalController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpHierarchicalController_VelLimitPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
  if(ctl.compPostureTask->eval().norm() < 0.01)
  {
    if(ctl.sequenceOutput.compare("FINISHED") == 0)
    {
      ctl.sequenceOutput = "B";
      ctl.velLimitCounter = 0;
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpHierarchicalController_VelLimitPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpHierarchicalController_VelLimitPose", RALExpHierarchicalController_VelLimitPose)
