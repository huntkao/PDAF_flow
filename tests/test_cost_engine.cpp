#include <gtest/gtest.h>
#include <algo/sad_cost_engine.h>

#include <algorithm>
#include <cmath>

using namespace pdaf;

// 產生 L=tex(i)、R=tex(i-d)*gain 的單 ROI frame
static PdFrame makeShiftedFrame(int d, float right_gain_err, int n = 64) {
  PdFrame f;
  RoiSamples r;
  r.width = n; r.height = 1;
  auto tex = [](float x) { return 2.f + std::sin(0.37f * x) + 0.5f * std::sin(1.13f * x); };
  for (int i = 0; i < n; ++i) {
    r.left.push_back(tex(static_cast<float>(i)));
    r.right.push_back(tex(static_cast<float>(i - d)) * right_gain_err);
  }
  f.rois.push_back(std::move(r));
  return f;
}

static int argmin(const std::vector<float>& v) {
  return static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin());
}

TEST(CostEngine, MinimumAtTrueShift) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -8, 8);
  auto out = m1.compute(makeShiftedFrame(3, 1.f));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].shift_min, -8);
  EXPECT_EQ(out[0].costs.size(), 17u);
  EXPECT_EQ(out[0].shift_min + argmin(out[0].costs), 3);
  EXPECT_GT(out[0].valid_samples, 0);
}

TEST(CostEngine, LrcCorrectionCancelsGainMismatch) {
  // R 通道 gain 高 20%；校正 right_gain=1/1.2 後極小值應回到真 shift 且 cost 近 0
  SadCostEngine m1;
  m1.init(LrcCalib{1.f, 1.f / 1.2f}, PdPatternDesc{}, -8, 8);
  auto out = m1.compute(makeShiftedFrame(-2, 1.2f));
  EXPECT_EQ(out[0].shift_min + argmin(out[0].costs), -2);
  EXPECT_LT(*std::min_element(out[0].costs.begin(), out[0].costs.end()), 0.05f);
}

TEST(CostEngine, MultiRoiProducesOneSequenceEach) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -4, 4);
  auto f = makeShiftedFrame(1, 1.f);
  f.rois.push_back(f.rois[0]);
  auto out = m1.compute(f);
  EXPECT_EQ(out.size(), 2u);
}

TEST(CostEngine, EmptyRoiGivesZeroValidSamples) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -4, 4);
  PdFrame f;
  f.rois.push_back(RoiSamples{});  // width=height=0
  auto out = m1.compute(f);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].valid_samples, 0);  // 不丟例外，降級
}
