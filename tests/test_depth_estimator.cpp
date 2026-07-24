#include <algo/parabolic_depth_estimator.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace pdaf;

static CostSequence makeCost(int shift_min, std::vector<float> costs, int valid = 100)
{
  return CostSequence{shift_min, std::move(costs), valid};
}

TEST(DepthEstimator, FindsIntegerMinimum)
{
  // 極小值在 shift=+2（對稱 → 無 sub-pixel 偏移）
  auto c = makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4});
  auto e = ParabolicDepthEstimator{}.estimate(c);
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 1.0f, 0.01f); // min at index 5 → shift -4+5=+1
  EXPECT_GT(e.confidence, 0.5f);
}

TEST(DepthEstimator, SubPixelInterpolation)
{
  // 拋物線 (s-0.5)^2 取樣於整數 shift → 內插應得 0.5
  std::vector<float> v;
  for (int s = -4; s <= 4; ++s)
  {
    v.push_back((s - 0.5f) * (s - 0.5f));
  }
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, v));
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 0.5f, 0.01f);
}

TEST(DepthEstimator, FlatCurveGivesZeroConfidence)
{
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, std::vector<float>(9, 5.f)));
  EXPECT_LT(e.confidence, 0.1f);
}

TEST(DepthEstimator, BoundaryMinimumHalvesConfidence)
{
  auto edge = ParabolicDepthEstimator{}.estimate(makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}));
  auto mid = ParabolicDepthEstimator{}.estimate(makeCost(-4, {2, 1, 0.2f, 1, 2, 4, 6, 8, 10}));
  EXPECT_TRUE(edge.valid);
  EXPECT_NEAR(edge.disparity, -4.f, 0.01f); // 邊界不內插
  EXPECT_LT(edge.confidence, mid.confidence);
}

TEST(DepthEstimator, NoValidSamplesIsInvalid)
{
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0));
  EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.confidence, 0.f);
}

TEST(DepthEstimator, BroadValleyPenalizedVsSharp)
{
  // 同一個最小值、同為單峰，但一個寬底一個窄底：曲率項讓窄谷信心較高、寬谷被折扣。
  // 兩者深度（c_min/mean）相近，舊式只看深度會給寬谷 > 0.8；曲率折扣後應 < 0.7。
  auto broad = ParabolicDepthEstimator{}.estimate(makeCost(-4, {9, 9, 9, 2, 1, 2, 9, 9, 9}));
  auto sharp = ParabolicDepthEstimator{}.estimate(makeCost(-4, {9, 9, 9, 8, 1, 8, 9, 9, 9}));
  EXPECT_TRUE(broad.valid);
  EXPECT_GT(sharp.confidence, broad.confidence);
  EXPECT_LT(broad.confidence, 0.7f);
}

TEST(DepthEstimator, SecondMinimumReducesConfidence)
{
  // 兩個勢均力敵的深谷（歧義／aliasing）→ 次低點檢查把信心壓到接近 0，
  // 即使主谷本身又深又尖。舊式只看深度會給雙峰 > 0.7。
  auto unimodal = ParabolicDepthEstimator{}.estimate(makeCost(-4, {8, 7, 5, 3, 1, 3, 5, 7, 8}));
  auto bimodal = ParabolicDepthEstimator{}.estimate(makeCost(-4, {8, 1.2f, 8, 3, 1, 3, 8, 1.3f, 8}));
  EXPECT_GT(unimodal.confidence, 0.5f);
  EXPECT_LT(bimodal.confidence, 0.15f);
  EXPECT_NEAR(bimodal.disparity, unimodal.disparity, 0.01f); // 主谷位置相同，差別只在歧義
}

TEST(DepthEstimator, WideUnimodalValleyNotTreatedAsAmbiguous)
{
  // 一個寬但單峰的谷（谷寬 > ±2 樣本，如模擬器的低頻紋理）：谷自己的坡不該被
  // 當成第二個谷。次低點檢查必須排除主谷整段單調 basin，只計被山脊隔開的競爭谷。
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-5, {8, 4, 1.6f, 0.8f, 0.4f, 0.2f, 0.4f, 0.8f, 1.6f, 4, 8}));
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 0.f, 0.01f); // 對稱谷，谷底在中央
  EXPECT_GT(e.confidence, 0.5f);        // 單峰寬谷仍應可信
}

TEST(DepthEstimatorTrace, ResultMatchesEstimate)
{
  ParabolicDepthEstimator m2;
  const std::vector<CostSequence> cases = {
      makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4}),    // 一般
      makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}), // 邊界
      makeCost(-4, std::vector<float>(9, 5.f)),        // 平（低 depth）
      makeCost(-4, {8, 1.2f, 8, 3, 1, 3, 8, 1.3f, 8}), // 雙峰
      makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0),    // no samples
  };
  for (const auto& c : cases)
  {
    const DepthEstimate a = m2.estimate(c);
    const DepthEstimate b = m2.estimateTraced(c).result;
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_FLOAT_EQ(a.disparity, b.disparity);
    EXPECT_FLOAT_EQ(a.confidence, b.confidence);
  }
}

TEST(DepthEstimatorTrace, GoldenUnimodal)
{
  // {8,6,4,2,1,0.2,1,2,4} shift_min -4：full basin、無競爭谷
  auto t = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4}));
  EXPECT_FALSE(t.degenerate_no_samples);
  EXPECT_FALSE(t.degenerate_flat);
  EXPECT_FALSE(t.boundary);
  EXPECT_EQ(t.mi, 5u);
  EXPECT_FLOAT_EQ(t.cmin, 0.2f);
  EXPECT_NEAR(t.mean, 3.1333f, 1e-3f);
  EXPECT_NEAR(t.depth, 0.9362f, 1e-3f);
  EXPECT_EQ(t.basin_lo, 0u);
  EXPECT_EQ(t.basin_hi, 8u);
  EXPECT_TRUE(std::isinf(t.second)); // 無競爭谷
  EXPECT_FLOAT_EQ(t.unamb, 1.f);
  EXPECT_FLOAT_EQ(t.c_m1, 1.f);
  EXPECT_FLOAT_EQ(t.c_0, 0.2f);
  EXPECT_FLOAT_EQ(t.c_p1, 1.f);
  EXPECT_NEAR(t.sharp, 0.8f, 1e-4f);
  EXPECT_NEAR(t.delta, 0.f, 1e-4f);
  EXPECT_NEAR(t.result.disparity, 1.f, 1e-4f);
  EXPECT_NEAR(t.result.confidence, 0.8426f, 1e-3f);
  EXPECT_TRUE(t.result.valid);
}

TEST(DepthEstimatorTrace, GoldenBimodalBasinAndSecond)
{
  // {8,1.2,8,3,1,3,8,1.3,8} shift_min -4：主谷 basin [2,6]、競爭谷 second=1.2
  auto t = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {8, 1.2f, 8, 3, 1, 3, 8, 1.3f, 8}));
  EXPECT_EQ(t.mi, 4u);
  EXPECT_EQ(t.basin_lo, 2u);
  EXPECT_EQ(t.basin_hi, 6u);
  EXPECT_FLOAT_EQ(t.second, 1.2f);
  EXPECT_NEAR(t.unamb, 0.0554f, 1e-3f);
  EXPECT_NEAR(t.result.disparity, 0.f, 1e-4f);
  EXPECT_LT(t.result.confidence, 0.15f);
}

TEST(DepthEstimatorTrace, DegenerateFlagsSet)
{
  auto ns = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0));
  EXPECT_TRUE(ns.degenerate_no_samples);
  EXPECT_FALSE(ns.result.valid);
  auto flat = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, std::vector<float>(9, 0.f)));
  EXPECT_TRUE(flat.degenerate_flat);
  EXPECT_FALSE(flat.result.valid);
  auto edge = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}));
  EXPECT_TRUE(edge.boundary);
  EXPECT_TRUE(edge.result.valid);
}
