#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/control/af_config.h>
#include <pdaf/control/focus_estimator.h>
#include <pdaf/hal/lens_actuator.h>

namespace pdaf
{

enum class AfState
{
  kIdle,
  kMeasuring,
  kMoving,
  kSettling,
  kVerifying,
  kFocused,
  kFailed
};
const char* toString(AfState s);

struct AfFrameLog
{
  uint64_t frame_id = 0;
  AfState state_before = AfState::kIdle;
  AfState state_after = AfState::kIdle;
  float disparity = 0.f;
  float confidence = 0.f;
  int lens_step = 0;
  int target_step = 0;
};

// 單次 AF 狀態機。逐 frame 由 onFrame() 驅動（產品 AF 的實際型態）。
// CAF 擴充點：kFocused 後加場景監測轉移即可，不需改架構。
class AfController
{
public:
  AfController(IFocusEstimator& estimator, ILensMapper& mapper,
               ILensActuator& actuator, const TuningConfig& tuning);
  void trigger();
  AfState state() const
  {
    return state_;
  }
  AfFrameLog onFrame(const AfRequest& request, const PdInput& input);

private:
  void handleMeasurement(const PdInput& input, AfFrameLog& log);

  IFocusEstimator& estimator_;
  ILensMapper& mapper_;
  ILensActuator& actuator_;
  TuningConfig tuning_;
  AfState state_ = AfState::kIdle;
  int retries_ = 0;
  int iterations_ = 0;
  int target_step_ = 0;
};

} // namespace pdaf
