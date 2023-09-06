#include "RALExpHierarchicalController_Initial.h"

#include <RALExpHierarchicalController/RALExpHierarchicalController.h>

void RALExpHierarchicalController_Initial::configure(const mc_rtc::Configuration & config) {}

void RALExpHierarchicalController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);

  if(ctl.sequenceOutput.compare("FINISHED") == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Fini");
  }

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RALExpHierarchicalController] Switched to Initial state - Position controlled");
}

bool RALExpHierarchicalController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);

  if(not ctl.waitingForInput)
  {
    output("OK");
    return true;
  }

  return false;
}

void RALExpHierarchicalController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RALExpHierarchicalController &>(ctl_);
}

EXPORT_SINGLE_STATE("RALExpHierarchicalController_Initial", RALExpHierarchicalController_Initial)
