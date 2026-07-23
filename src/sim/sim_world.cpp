#include "sim/sim_world.h"

#include <algorithm>
#include <cmath>

namespace pdaf {

SimWorld::SimWorld(const SensorConfig& sensor, const SimConfig& sim, const DccTable& dcc)
    : sensor_(sensor), sim_(sim), dcc_(dcc),
      current_step_(sim.initial_step), target_step_(sim.initial_step),
      rng_(sim.seed) {
  std::uniform_real_distribution<float> u(0.f, 6.28f);
  for (auto& p : phase_) p = u(rng_);
}

int SimWorld::inFocusStep() const {
  const int s = sim_.step_inf +
      static_cast<int>(std::lround(sim_.focus_gain / sim_.object_distance_mm));
  return std::clamp(s, dcc_.step_min, dcc_.step_max);
}

float SimWorld::groundTruthDisparity() const {
  return (inFocusStep() - current_step_) / dccInterp(dcc_, current_step_);
}

void SimWorld::moveTo(int step) {
  target_step_ = std::clamp(step, dcc_.step_min, dcc_.step_max);
  if (target_step_ != current_step_) settle_counter_ = sim_.settle_frames;
}

LensStatus SimWorld::lensStatus() const { return {current_step_, settle_counter_ > 0}; }

float SimWorld::texture(float x) const {
  // 頻率皆低於 π/16 (~0.196 rad/sample)，確保 SAD cost 在 ±16 shift 搜尋窗內單峰、
  // 不與搜尋窗產生 aliasing（高頻紋理的週期若 ≈ 搜尋窗寬會出現假極小值）
  return 2.f + 0.7f * std::sin(0.07f * x + phase_[0]) +
         0.6f * std::sin(0.13f * x + phase_[1]) +
         0.4f * std::sin(0.19f * x + phase_[2]);
}

float SimWorld::blurredTexture(float x, int radius) const {
  if (radius <= 0) return texture(x);
  float acc = 0.f;
  for (int j = -radius; j <= radius; ++j) acc += texture(x + static_cast<float>(j));
  return acc / static_cast<float>(2 * radius + 1);
}

PdInput SimWorld::capture(const AfRequest& request) {
  // frame 前進：actuator 運動
  if (settle_counter_ > 0 && --settle_counter_ == 0) current_step_ = target_step_;

  const float d = groundTruthDisparity();
  const int defocus = std::abs(inFocusStep() - current_step_);
  const int radius = std::min(6, defocus / 25);
  std::normal_distribution<float> noise(0.f, sim_.noise_sigma);

  PdInput in;
  in.meta = {frame_id_++, static_cast<double>(frame_id_) * 33.3, current_step_};

  PdFrame f;
  f.meta = in.meta;
  f.pattern = sensor_.pattern;
  const int n = sensor_.roi_sample_width;
  const int h = sensor_.roi_sample_height;
  const size_t roi_count = std::max<size_t>(1, request.rois.size());
  for (size_t r = 0; r < roi_count; ++r) {
    RoiSamples s;
    s.width = n;
    s.height = h;
    const float roi_offset = static_cast<float>(r) * 131.f;  // 各 ROI 看不同紋理區
    for (int y = 0; y < h; ++y) {
      for (int i = 0; i < n; ++i) {
        const float x = static_cast<float>(i) + static_cast<float>(y) * 7.f + roi_offset;
        s.left.push_back(blurredTexture(x, radius));
        s.right.push_back(blurredTexture(x - d, radius) * sim_.gain_mismatch + noise(rng_));
      }
    }
    f.rois.push_back(std::move(s));
  }
  in.raw = std::move(f);
  return in;
}

}  // namespace pdaf
