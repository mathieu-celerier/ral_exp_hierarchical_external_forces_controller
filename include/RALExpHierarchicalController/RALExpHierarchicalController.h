#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>

#include <RALExpHierarchicalController/api.h>

#define HIGH_RESIDUAL_GAIN 10.0
#define LOW_RESIDUAL_GAIN 0.5

struct RALExpHierarchicalController_DLLAPI RALExpHierarchicalController : public mc_control::fsm::Controller
{
  RALExpHierarchicalController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> eeTask;

  bool moveNextState;
  std::map<std::string, std::vector<double>> postureVelLimit;
  std::map<std::string, std::vector<double>> postureJointLim;
  std::map<std::string, std::vector<double>> postureTarget;
  Eigen::VectorXd posture_target_log;

  int velLimitCount;
  double velLimitDuration;

  int jointLimitCount;
  double jointLimitDuration;

  int velLimitCounter;
  int jointLimitCounter;

  int currentSequence;
  std::string sequenceOutput;
  bool waitingForInput;

private:
  mc_rtc::Configuration config_;
  void getPostureTarget(void);
  int stateIndex_;
};
