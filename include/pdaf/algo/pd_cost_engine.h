#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf
{

// LRC/SPC 校正資料（參考實作用全域 gain；正式版可擴充為 per-position 表）
struct LrcCalib
{
  float left_gain = 1.f;
  float right_gain = 1.f;
};

// M1：套用 LRC/SPC 校正，計算各 ROI 的 matching cost sequence
class IPdCostEngine
{
public:
  virtual ~IPdCostEngine() = default;
  virtual void init(const LrcCalib& calib, const PdPatternDesc& pattern,
                    int shift_min, int shift_max) = 0;
  virtual std::vector<CostSequence> compute(const PdFrame& frame) = 0;
};

} // namespace pdaf
