#pragma once

#include <mc_control/fsm/State.h>

struct RALExpHierarchicalController_VelLimitEF : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double elapsedTime_;
  double jointVel;
  double upperLimit;
  double lowerLimit;
  double maxLimitCross_;
};