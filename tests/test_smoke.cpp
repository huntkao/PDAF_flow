#include <gtest/gtest.h>
#include <pdaf/types.h>

TEST(Smoke, TypesCompileAndDefault)
{
  pdaf::PdInput in;
  EXPECT_FALSE(in.raw.has_value());
  EXPECT_FALSE(in.hw_costs.has_value());
  pdaf::DepthEstimate e;
  EXPECT_FALSE(e.valid);
}
