#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/control/af_config.h>
#include <pdaf/hal/lens_actuator.h>
#include <pdaf/hal/pd_data_source.h>
#include <pdaf/types.h>

#include <random>

namespace pdaf {

// 薄透鏡閉環模擬：VCM step ↔ defocus ↔ L/R 相位偏移
class SimWorld {
 public:
  SimWorld(const SensorConfig& sensor, const SimConfig& sim, const DccTable& dcc);
  PdInput capture(const AfRequest& request);  // 每次呼叫前進一個 frame
  void moveTo(int step);
  LensStatus lensStatus() const;
  int inFocusStep() const;
  int currentStep() const { return current_step_; }
  float groundTruthDisparity() const;

 private:
  float texture(float x) const;      // 連續紋理（seed 決定相位）
  float blurredTexture(float x, int radius) const;

  SensorConfig sensor_;
  SimConfig sim_;
  DccTable dcc_;
  int current_step_;
  int target_step_;
  int settle_counter_ = 0;
  uint64_t frame_id_ = 0;
  float phase_[3];
  mutable std::mt19937 rng_;
};

class SimPdDataSource : public IPdDataSource {
 public:
  explicit SimPdDataSource(SimWorld& world) : world_(world) {}
  PdInput capture(const AfRequest& request) override { return world_.capture(request); }

 private:
  SimWorld& world_;
};

class SimLensActuator : public ILensActuator {
 public:
  explicit SimLensActuator(SimWorld& world) : world_(world) {}
  void moveTo(int step) override { world_.moveTo(step); }
  LensStatus getStatus() const override { return world_.lensStatus(); }

 private:
  SimWorld& world_;
};

}  // namespace pdaf
