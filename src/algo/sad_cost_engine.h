#pragma once
#include <pdaf/algo/pd_cost_engine.h>

namespace pdaf
{
// 參考實作：gain 校正後逐 shift 計算 mean SAD
class SadCostEngine : public IPdCostEngine
{
public:
  void init(const LrcCalib& calib, const PdPatternDesc& pattern,
            int shift_min, int shift_max) override;
  std::vector<CostSequence> compute(const PdFrame& frame) override;

private:
  LrcCalib calib_;
  int shift_min_ = -8;
  int shift_max_ = 8;
};
} // namespace pdaf
