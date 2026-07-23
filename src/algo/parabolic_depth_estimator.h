#pragma once
#include <pdaf/algo/depth_estimator.h>

namespace pdaf
{
// 參考實作：cost 極小值 + 三點拋物線內插；confidence 由曲線相對深度導出
class ParabolicDepthEstimator : public IDepthEstimator
{
public:
  DepthEstimate estimate(const CostSequence& cost) override;
};
} // namespace pdaf
