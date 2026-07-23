#pragma once
#include <pdaf/hal/lens_actuator.h>

#include <vector>

namespace pdaf
{
// replay 模式：只記錄命令，瞬移、永不 moving（不影響重播資料）
class NullLensActuator : public ILensActuator
{
public:
  explicit NullLensActuator(int initial_step = 0) : step_(initial_step)
  {
  }
  void moveTo(int step) override
  {
    step_ = step;
    history_.push_back(step);
  }
  LensStatus getStatus() const override
  {
    return {step_, false};
  }
  const std::vector<int>& history() const
  {
    return history_;
  }

private:
  int step_;
  std::vector<int> history_;
};
} // namespace pdaf
