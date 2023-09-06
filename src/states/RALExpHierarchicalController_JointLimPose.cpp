#include "RALExpHierarchicalController_JointLimPose.h"

#include <RALExpHierarchicalController/RALExpHierarchicalController.h>

void RALExpHierarchicalController_JointLimPose::configure(const mc_rtc::Configuration & config) {}

void RALExpHierarchicalController_JointLimPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.6, false, true));
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
  ctl.compPostureTask->target(ctl.postureJointLim);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.eeTask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RALExpHierarchicalController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpHierarchicalController_JointLimPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
  if(ctl.compPostureTask->eval().norm() < 0.001)
  {
    if(ctl.sequenceOutput.compare("FINISHED") == 0)
    {
      if(ctl.jointLimitCounter > 0)
      {
        ctl.sequenceOutput = "B";
        ctl.jointLimitCounter = 0;
      }
      else
      {
        ctl.sequenceOutput = "A";
      }
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpHierarchicalController_JointLimPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpHierarchicalController_JointLimPose", RALExpHierarchicalController_JointLimPose)
