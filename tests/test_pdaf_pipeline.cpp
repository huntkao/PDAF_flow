#include <control/pdaf_pipeline.h>
#include <gtest/gtest.h>

#include <algorithm>

using namespace pdaf;

namespace
{

// stub M1：回傳固定 cost；記錄是否被呼叫
class StubCostEngine : public IPdCostEngine
{
public:
  bool called = false;
  void init(const LrcCalib&, const PdPatternDesc&, int, int) override
  {
  }
  std::vector<CostSequence> compute(const PdFrame& f) override
  {
    called = true;
    return std::vector<CostSequence>(f.rois.size(), CostSequence{-2, {3, 1, 3, 5, 7}, 10});
  }
};

// stub M2：disparity = shift 極小值位置
class StubEstimator : public IDepthEstimator
{
public:
  DepthEstimate estimate(const CostSequence& c) override
  {
    auto mi = std::min_element(c.costs.begin(), c.costs.end()) - c.costs.begin();
    return {static_cast<float>(c.shift_min + mi), 0.9f, true};
  }
};

} // namespace

TEST(PdafPipeline, RawPathRunsM1ThenM2)
{
  auto m1 = std::make_unique<StubCostEngine>();
  auto* m1p = m1.get();
  PdafPipeline pipe(std::move(m1), std::make_unique<StubEstimator>());
  PdInput in;
  PdFrame f;
  f.rois.resize(2);
  in.raw = f;
  auto out = pipe.process(in);
  EXPECT_TRUE(m1p->called);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_FLOAT_EQ(out[0].disparity, -1.f); // min at index 1 → -2+1
}

TEST(PdafPipeline, HwCostPathBypassesM1)
{
  auto m1 = std::make_unique<StubCostEngine>();
  auto* m1p = m1.get();
  PdafPipeline pipe(std::move(m1), std::make_unique<StubEstimator>());
  PdInput in;
  in.hw_costs = std::vector<CostSequence>{CostSequence{0, {5, 1, 5}, 10}};
  auto out = pipe.process(in);
  EXPECT_FALSE(m1p->called);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].disparity, 1.f);
}

TEST(PdafPipeline, EmptyInputGivesNoEstimates)
{
  PdafPipeline pipe(std::make_unique<StubCostEngine>(), std::make_unique<StubEstimator>());
  EXPECT_TRUE(pipe.process(PdInput{}).empty()); // 不丟例外
}
