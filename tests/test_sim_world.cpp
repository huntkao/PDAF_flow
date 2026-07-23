#include <gtest/gtest.h>
#include <sim/sim_world.h>

#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>

#include <cmath>

using namespace pdaf;

namespace {

SensorConfig sensorCfg() {
  SensorConfig s;
  s.roi_sample_width = 64;
  s.roi_sample_height = 4;
  return s;
}

SimConfig simCfg(double dist_mm, int init_step) {
  SimConfig c;
  c.object_distance_mm = dist_mm;
  c.initial_step = init_step;
  c.noise_sigma = 0.005f;
  c.gain_mismatch = 1.1f;
  c.settle_frames = 3;
  c.seed = 42;
  return c;  // step_inf=100, focus_gain=150000（預設）
}

DccTable dcc() { return DccTable{0, 1023, {{0, 50.f}, {1023, 50.f}}}; }

}  // namespace

TEST(SimWorld, InFocusStepFromDistance) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  EXPECT_EQ(w.inFocusStep(), 175);  // 100 + 150000/2000
  EXPECT_EQ(w.currentStep(), 300);
  EXPECT_NEAR(w.groundTruthDisparity(), (175 - 300) / 50.f, 0.01f);
}

TEST(SimWorld, ActuatorSettlesAfterConfiguredFrames) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  w.moveTo(500);
  AfRequest req{{Roi{0, 0, 64, 4}}};
  EXPECT_TRUE(w.lensStatus().moving);
  w.capture(req);
  w.capture(req);
  EXPECT_TRUE(w.lensStatus().moving);
  w.capture(req);  // 第 settle_frames 次 → 到位
  EXPECT_FALSE(w.lensStatus().moving);
  EXPECT_EQ(w.currentStep(), 500);
}

TEST(SimWorld, CaptureMetaCarriesFrameIdAndExposureStep) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  AfRequest req{{Roi{0, 0, 64, 4}}};
  auto a = w.capture(req);
  auto b = w.capture(req);
  EXPECT_EQ(b.meta.frame_id, a.meta.frame_id + 1);
  EXPECT_EQ(a.meta.lens_step_at_exposure, 300);
  ASSERT_TRUE(a.raw.has_value());
  ASSERT_EQ(a.raw->rois.size(), 1u);
  EXPECT_EQ(a.raw->rois[0].width, 64);
}

TEST(SimWorld, PipelineMeasuresGroundTruthDisparity) {
  // 模擬器資料丟進 M1+M2，量到的 disparity 應接近真值
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());  // d_gt = -2.5
  AfRequest req{{Roi{0, 0, 64, 4}}};
  auto in = w.capture(req);

  SadCostEngine m1;
  m1.init(LrcCalib{1.f, 1.f / 1.1f}, PdPatternDesc{}, -16, 16);  // 校正抵銷 gain_mismatch
  auto costs = m1.compute(*in.raw);
  auto e = ParabolicDepthEstimator{}.estimate(costs[0]);
  EXPECT_TRUE(e.valid);
  EXPECT_GT(e.confidence, 0.3f);
  EXPECT_NEAR(e.disparity, w.groundTruthDisparity(), 0.5f);
}
