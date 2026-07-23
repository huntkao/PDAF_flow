#include <gtest/gtest.h>

#include <algo/dcc_lens_mapper.h>
#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>
#include <control/pdaf_pipeline.h>
#include <pdaf/control/af_controller.h>
#include <sim/sim_world.h>

#include <cmath>

using namespace pdaf;

namespace {

struct RunResult {
  AfState final_state;
  int frames_used;
  int final_error_steps;
};

// 框架的「活規格」：模擬器 + 全 pipeline 閉環
RunResult runScenario(double dist_mm, int init_step, float noise_sigma) {
  SensorConfig sensor;
  sensor.roi_sample_width = 64;
  sensor.roi_sample_height = 4;

  SimConfig sim;
  sim.object_distance_mm = dist_mm;
  sim.initial_step = init_step;
  sim.noise_sigma = noise_sigma;
  sim.gain_mismatch = 1.1f;
  sim.settle_frames = 3;
  sim.seed = 42;

  DccTable dcc{0, 1023, {{0, 50.f}, {1023, 50.f}}};
  TuningConfig tun{-16, 16, 0.3f, 0.25f, 3, 6};

  SimWorld world(sensor, sim, dcc);
  SimPdDataSource source(world);
  SimLensActuator actuator(world);

  auto m1 = std::make_unique<SadCostEngine>();
  m1->init(LrcCalib{1.f, 1.f / 1.1f}, sensor.pattern, tun.shift_min, tun.shift_max);
  PdafPipeline pipeline(std::move(m1), std::make_unique<ParabolicDepthEstimator>());
  DccLensMapper mapper;
  mapper.init(dcc);

  AfController ctrl(pipeline, mapper, actuator, tun);
  ctrl.trigger();

  AfRequest req{{Roi{0, 0, 256, 128}}};
  int frames = 0;
  for (; frames < 100; ++frames) {
    auto in = source.capture(req);
    ctrl.onFrame(req, in);
    if (ctrl.state() == AfState::kFocused || ctrl.state() == AfState::kFailed) break;
  }
  return {ctrl.state(), frames + 1,
          std::abs(world.currentStep() - world.inFocusStep())};
}

}  // namespace

TEST(ClosedLoop, MidDistanceConverges) {
  auto r = runScenario(2000.0, 300, 0.01f);  // in-focus=175, d0=-2.5
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);  // 0.25 disparity*50 steps + 量測 sub-pixel 誤差
  EXPECT_LT(r.frames_used, 40);
}

TEST(ClosedLoop, NearDistanceConverges) {
  auto r = runScenario(300.0, 300, 0.01f);  // in-focus=600, d0=+6
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);
  EXPECT_LT(r.frames_used, 40);
}

TEST(ClosedLoop, FarDistanceLargeDefocusConverges) {
  auto r = runScenario(5000.0, 600, 0.01f);  // in-focus=130, d0=-9.4
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);
  EXPECT_LT(r.frames_used, 40);
}

TEST(ClosedLoop, AlreadyInFocusFinishesImmediately) {
  auto r = runScenario(2000.0, 175, 0.005f);  // 初始即合焦
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.frames_used, 5);
}
