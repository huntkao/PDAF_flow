#include <gtest/gtest.h>
#include <algo/dcc_lens_mapper.h>

using namespace pdaf;

static DccTable twoAnchor() {
  return DccTable{0, 1023, {{0, 40.f}, {1000, 60.f}}};
}

TEST(LensMapper, DccInterpolatesBetweenAnchors) {
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 500), 50.f);
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 0), 40.f);
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 1023), 60.f);  // 超出末錨點 → clamp
}

TEST(LensMapper, TargetUsesExposureStepAsBase) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  // 曝光當下 step=500（dcc=50），disparity=+2 → 500 + 2*50 = 600
  auto cmd = m3.toLensCommand(DepthEstimate{2.f, 0.9f, true}, 500);
  EXPECT_EQ(cmd.target_step, 600);
}

TEST(LensMapper, TargetClampedToStepRange) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  auto cmd = m3.toLensCommand(DepthEstimate{-100.f, 0.9f, true}, 100);
  EXPECT_EQ(cmd.target_step, 0);
  cmd = m3.toLensCommand(DepthEstimate{100.f, 0.9f, true}, 900);
  EXPECT_EQ(cmd.target_step, 1023);
}

TEST(LensMapper, ToleranceScalesWithDcc) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  auto cmd = m3.toLensCommand(DepthEstimate{1.f, 0.9f, true}, 500);
  EXPECT_GT(cmd.tolerance, 0);  // 約 0.25 disparity 對應的 step 數
  EXPECT_LE(cmd.tolerance, 20);
}
