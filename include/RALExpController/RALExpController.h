#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>

#include <RALExpController/api.h>

#define HIGH_RESIDUAL_GAIN 10.0
#define LOW_RESIDUAL_GAIN 0.5

struct RALExpController_DLLAPI RALExpController : public mc_control::fsm::Controller
{
  RALExpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> eeTask;

  bool moveNextState;
  std::map<std::string, std::vector<double>> postureVelLimit;
  std::map<std::string, std::vector<double>> postureTarget;
  Eigen::VectorXd posture_target_log;

private:
  mc_rtc::Configuration config_;
  void getPostureTarget(void);
  int stateIndex_;
};
