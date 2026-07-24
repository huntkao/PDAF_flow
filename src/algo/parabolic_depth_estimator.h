#pragma once
#include <pdaf/algo/depth_estimator.h>

#include <cstddef>

namespace pdaf
{
// estimateTraced() 記錄的中間值，供視覺化/驗證工具標示（與生產路徑同一份計算）
struct DepthEstimateTrace
{
  DepthEstimate result;

  bool degenerate_no_samples = false; // v.size()<3 或 valid_samples<=0
  bool degenerate_flat = false;       // mean < 1e-6
  bool boundary = false;              // 最小點在搜尋邊界

  std::size_t mi = 0;
  float cmin = 0.f;
  float mean = 0.f;
  float depth = 0.f;
  std::size_t basin_lo = 0, basin_hi = 0;
  float second = 0.f; // basin 外最低成本；無競爭谷時為 +inf
  float unamb = 0.f;

  // interior（非邊界）才有效
  float c_m1 = 0.f, c_0 = 0.f, c_p1 = 0.f;
  float sharp = 0.f;
  float delta = 0.f;
};

// 參考實作：cost 極小值 + 三點拋物線內插；confidence = depth × unamb × (0.5+0.5·sharp)
class ParabolicDepthEstimator : public IDepthEstimator
{
public:
  DepthEstimate estimate(const CostSequence& cost) override;
  DepthEstimateTrace estimateTraced(const CostSequence& cost) const;
};
} // namespace pdaf
