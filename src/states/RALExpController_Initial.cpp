#include "RALExpController_Initial.h"

#include <RALExpController/RALExpController.h>

void RALExpController_Initial::configure(const mc_rtc::Configuration & config) {}

void RALExpController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RALExpController] Switched to Initial state - Position controlled");
}

bool RALExpController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
  output("OK");
  return true;
}

void RALExpController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpController_Initial", RALExpController_Initial)
