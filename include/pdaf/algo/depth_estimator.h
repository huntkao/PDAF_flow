#pragma once
#include <pdaf/types.h>

namespace pdaf {
// M2：由 cost sequence 推估 disparity 與 confidence
class IDepthEstimator {
 public:
  virtual ~IDepthEstimator() = default;
  virtual DepthEstimate estimate(const CostSequence& cost) = 0;
};
}  // namespace pdaf
