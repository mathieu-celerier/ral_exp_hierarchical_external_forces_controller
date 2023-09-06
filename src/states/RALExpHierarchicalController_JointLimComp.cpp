#include "RALExpHierarchicalController_JointLimComp.h"

#include <mc_tvm/Robot.h>
#include <RALExpHierarchicalController/RALExpHierarchicalController.h>

void RALExpHierarchicalController_JointLimComp::configure(const mc_rtc::Configuration & config) {}

void RALExpHierarchicalController_JointLimComp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);

  // Modify robot module to add limits to 3rd joint, to chowcase compliance and safety on joint limits
  auto & tvm_robot = ctl.robot().tvmRobot();
  tvm_robot.limits().ql[2] = -1.57;
  tvm_robot.limits().qu[2] = 1.57;

  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.6, false, true));
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);

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
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(10.0);
  ctl.compPostureTask->target(ctl.postureJointLim);
  ctl.compPostureTask->makeCompliant(true);
  ctl.solver().removeTask(ctl.eeTask);

  elapsedTime_ = 0;
  ctl.jointLimitCounter++;

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("[RALExpHierarchicalController] Switched to Sensor Testing state - Position controlled");
}

bool RALExpHierarchicalController_JointLimComp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
  elapsedTime_ += ctl.timeStep;

  if(elapsedTime_ >= ctl.jointLimitDuration)
  {
    if(ctl.jointLimitCounter > ctl.jointLimitCount)
    {
      ctl.sequenceOutput = "FINISHED";
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void RALExpHierarchicalController_JointLimComp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpHierarchicalController_JointLimComp", RALExpHierarchicalController_JointLimComp)
