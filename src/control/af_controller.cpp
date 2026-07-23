#include <pdaf/control/af_controller.h>

#include <cmath>

namespace pdaf
{

const char* toString(AfState s)
{
  switch (s)
  {
  case AfState::kIdle:
    return "IDLE";
  case AfState::kMeasuring:
    return "MEASURING";
  case AfState::kMoving:
    return "MOVING";
  case AfState::kSettling:
    return "SETTLING";
  case AfState::kVerifying:
    return "VERIFYING";
  case AfState::kFocused:
    return "FOCUSED";
  case AfState::kFailed:
    return "FAILED";
  }
  return "?";
}

AfController::AfController(IFocusEstimator& estimator, ILensMapper& mapper,
                           ILensActuator& actuator, const TuningConfig& tuning)
    : estimator_(estimator), mapper_(mapper), actuator_(actuator), tuning_(tuning)
{
}

void AfController::trigger()
{
  if (state_ == AfState::kIdle || state_ == AfState::kFocused ||
      state_ == AfState::kFailed)
  {
    retries_ = 0;
    iterations_ = 0;
    state_ = AfState::kMeasuring;
  }
}

AfFrameLog AfController::onFrame(const AfRequest&, const PdInput& input)
{
  AfFrameLog log;
  log.frame_id = input.meta.frame_id;
  log.state_before = state_;
  log.lens_step = actuator_.getStatus().current_step;
  log.target_step = target_step_;

  switch (state_)
  {
  case AfState::kIdle:
  case AfState::kFocused:
  case AfState::kFailed:
    break;
  case AfState::kMoving:
    state_ = AfState::kSettling;
    [[fallthrough]];
  case AfState::kSettling:
    if (!actuator_.getStatus().moving)
    {
      state_ = AfState::kVerifying;
    }
    break;
  case AfState::kMeasuring:
  case AfState::kVerifying:
    handleMeasurement(input, log);
    break;
  }

  log.state_after = state_;
  log.target_step = target_step_;
  return log;
}

void AfController::handleMeasurement(const PdInput& input, AfFrameLog& log)
{
  const auto ests = estimator_.process(input);
  const DepthEstimate e = ests.empty() ? DepthEstimate{} : ests.front(); // primary ROI
  log.disparity = e.disparity;
  log.confidence = e.confidence;

  if (!e.valid || e.confidence < tuning_.confidence_threshold)
  {
    if (++retries_ > tuning_.max_retries)
    {
      state_ = AfState::kFailed;
    }
    return; // 低信心：留在原狀態重量測（執行期不丟例外，走降級路徑）
  }
  retries_ = 0;

  if (std::abs(e.disparity) < tuning_.in_focus_disparity)
  {
    state_ = AfState::kFocused;
    return;
  }
  if (++iterations_ > tuning_.max_iterations)
  {
    state_ = AfState::kFailed;
    return;
  }
  // 基準必須用曝光當下的 lens 位置（實機 stats 有 pipeline 延遲）
  const LensCommand cmd = mapper_.toLensCommand(e, input.meta.lens_step_at_exposure);
  target_step_ = cmd.target_step;
  actuator_.moveTo(cmd.target_step);
  state_ = AfState::kMoving;
}

} // namespace pdaf
