#include "algo/sad_cost_engine.h"

#include <cmath>

namespace pdaf {

void SadCostEngine::init(const LrcCalib& calib, const PdPatternDesc&,
                         int shift_min, int shift_max) {
  calib_ = calib;
  shift_min_ = shift_min;
  shift_max_ = shift_max;
}

std::vector<CostSequence> SadCostEngine::compute(const PdFrame& frame) {
  std::vector<CostSequence> out;
  for (const auto& r : frame.rois) {
    CostSequence cs;
    cs.shift_min = shift_min_;
    const int n = r.width;
    for (int s = shift_min_; s <= shift_max_; ++s) {
      double acc = 0.0;
      int cnt = 0;
      for (int y = 0; y < r.height; ++y) {
        for (int i = 0; i < n; ++i) {
          const int j = i + s;
          if (j < 0 || j >= n) continue;
          const float l = r.left[y * n + i] * calib_.left_gain;
          const float rr = r.right[y * n + j] * calib_.right_gain;
          acc += std::abs(l - rr);
          ++cnt;
        }
      }
      cs.costs.push_back(cnt > 0 ? static_cast<float>(acc / cnt) : 0.f);
    }
    cs.valid_samples = n * r.height;
    out.push_back(std::move(cs));
  }
  return out;
}

}  // namespace pdaf
