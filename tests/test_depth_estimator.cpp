#include <gtest/gtest.h>
#include <algo/parabolic_depth_estimator.h>

using namespace pdaf;

static CostSequence makeCost(int shift_min, std::vector<float> costs, int valid = 100) {
  return CostSequence{shift_min, std::move(costs), valid};
}

TEST(DepthEstimator, FindsIntegerMinimum) {
  // 極小值在 shift=+2（對稱 → 無 sub-pixel 偏移）
  auto c = makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4});
  auto e = ParabolicDepthEstimator{}.estimate(c);
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 1.0f, 0.01f);  // min at index 5 → shift -4+5=+1
  EXPECT_GT(e.confidence, 0.5f);
}

TEST(DepthEstimator, SubPixelInterpolation) {
  // 拋物線 (s-0.5)^2 取樣於整數 shift → 內插應得 0.5
  std::vector<float> v;
  for (int s = -4; s <= 4; ++s) v.push_back((s - 0.5f) * (s - 0.5f));
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, v));
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 0.5f, 0.01f);
}

TEST(DepthEstimator, FlatCurveGivesZeroConfidence) {
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, std::vector<float>(9, 5.f)));
  EXPECT_LT(e.confidence, 0.1f);
}

TEST(DepthEstimator, BoundaryMinimumHalvesConfidence) {
  auto edge = ParabolicDepthEstimator{}.estimate(makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}));
  auto mid  = ParabolicDepthEstimator{}.estimate(makeCost(-4, {2, 1, 0.2f, 1, 2, 4, 6, 8, 10}));
  EXPECT_TRUE(edge.valid);
  EXPECT_NEAR(edge.disparity, -4.f, 0.01f);  // 邊界不內插
  EXPECT_LT(edge.confidence, mid.confidence);
}

TEST(DepthEstimator, NoValidSamplesIsInvalid) {
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0));
  EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.confidence, 0.f);
}
