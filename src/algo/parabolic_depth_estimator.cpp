#include "algo/parabolic_depth_estimator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace pdaf {

DepthEstimate ParabolicDepthEstimator::estimate(const CostSequence& cost) {
  DepthEstimate e;
  const auto& v = cost.costs;
  if (v.size() < 3 || cost.valid_samples <= 0) return e;

  const size_t mi = std::min_element(v.begin(), v.end()) - v.begin();
  const float mean = std::accumulate(v.begin(), v.end(), 0.f) / v.size();
  if (mean < 1e-6f) return e;  // 全平：無紋理可匹配

  const float conf = std::clamp(1.f - v[mi] / mean, 0.f, 1.f);
  if (mi == 0 || mi + 1 == v.size()) {
    // 極小值在搜尋邊界：真值可能在範圍外，不內插且信心減半
    e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)), conf * 0.5f, true};
    return e;
  }
  const float c0 = v[mi - 1], c1 = v[mi], c2 = v[mi + 1];
  const float denom = c0 - 2.f * c1 + c2;
  const float delta = denom > 1e-9f ? 0.5f * (c0 - c2) / denom : 0.f;
  e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)) + delta, conf, true};
  return e;
}

}  // namespace pdaf
