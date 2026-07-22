#include "algo/dcc_lens_mapper.h"

#include <algorithm>
#include <cmath>

namespace pdaf {

float dccInterp(const DccTable& table, int step) {
  const auto& a = table.anchors;
  if (a.empty()) return 50.f;
  if (step <= a.front().step) return a.front().steps_per_disparity;
  if (step >= a.back().step) return a.back().steps_per_disparity;
  for (size_t i = 1; i < a.size(); ++i) {
    if (step <= a[i].step) {
      const float t = static_cast<float>(step - a[i - 1].step) /
                      static_cast<float>(a[i].step - a[i - 1].step);
      return a[i - 1].steps_per_disparity +
             t * (a[i].steps_per_disparity - a[i - 1].steps_per_disparity);
    }
  }
  return a.back().steps_per_disparity;
}

void DccLensMapper::init(const DccTable& table) { table_ = table; }

LensCommand DccLensMapper::toLensCommand(const DepthEstimate& est,
                                         int lens_step_at_exposure) {
  const float k = dccInterp(table_, lens_step_at_exposure);
  int target = static_cast<int>(std::lround(lens_step_at_exposure + est.disparity * k));
  target = std::clamp(target, table_.step_min, table_.step_max);
  const int tol = static_cast<int>(std::ceil(std::abs(k) * 0.25f));
  return {target, tol};
}

}  // namespace pdaf
