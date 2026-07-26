#include "algo/parabolic_depth_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace pdaf
{

DepthEstimateTrace ParabolicDepthEstimator::estimateTraced(const CostSequence& cost) const
{
  DepthEstimateTrace t;
  const auto& v = cost.costs;
  if (v.size() < 3 || cost.valid_samples <= 0)
  {
    t.degenerate_no_samples = true;
    return t;
  }

  const size_t n = v.size();
  t.mi = std::min_element(v.begin(), v.end()) - v.begin();
  t.mean = std::accumulate(v.begin(), v.end(), 0.f) / n;
  if (t.mean < 1e-6f)
  {
    t.degenerate_flat = true; // 全平：無紋理可匹配
    return t;
  }
  t.cmin = v[t.mi];

  // depth：谷底相對整體平均的深度，除以 mean 使其對場景對比尺度不變
  t.depth = std::clamp(1.f - t.cmin / t.mean, 0.f, 1.f);

  // 次低點檢查：排除主谷「整段單調 basin」，只有被山脊隔開的競爭谷才扣分
  size_t lo = t.mi;
  while (lo > 0 && v[lo - 1] >= v[lo])
  {
    --lo;
  }
  size_t hi = t.mi;
  while (hi + 1 < n && v[hi + 1] >= v[hi])
  {
    ++hi;
  }
  t.basin_lo = lo;
  t.basin_hi = hi;
  t.second = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < n; ++i)
  {
    if (i < lo || i > hi)
    {
      t.second = std::min(t.second, v[i]);
    }
  }
  const float depth_abs = t.mean - t.cmin;
  t.unamb = (std::isinf(t.second) || depth_abs < 1e-9f)
                ? 1.f
                : std::clamp((t.second - t.cmin) / depth_abs, 0.f, 1.f);

  // count_near：basin 外的局部極小值裡，有幾個落在 second 的 10% 容忍帶內
  // （容忍帶用相對值，跟 depth 正規化一樣對場景對比尺度不變）
  if (!std::isinf(t.second))
  {
    constexpr float kCountNearTolerance = 0.10f;
    const float band = t.second * (1.f + kCountNearTolerance);
    for (size_t i = 0; i < n; ++i)
    {
      if (i < lo || i > hi)
      {
        const bool is_local_min = (i == 0 || v[i - 1] >= v[i]) && (i + 1 == n || v[i + 1] >= v[i]);
        if (is_local_min && v[i] <= band)
        {
          ++t.count_near;
        }
      }
    }
  }

  if (t.mi == 0 || t.mi + 1 == n)
  {
    // 邊界：真值可能在範圍外，不內插；曲率未定義，信心對折並套用歧義因子
    t.boundary = true;
    t.result = {static_cast<float>(cost.shift_min + static_cast<int>(t.mi)), t.depth * t.unamb * 0.5f, true};
    return t;
  }
  t.c_m1 = v[t.mi - 1];
  t.c_0 = v[t.mi];
  t.c_p1 = v[t.mi + 1];
  const float denom = t.c_m1 - 2.f * t.c_0 + t.c_p1;
  t.delta = denom > 1e-9f ? 0.5f * (t.c_m1 - t.c_p1) / denom : 0.f;

  // 曲率／尖銳度：正規化二階差分（= 局部鄰點對比）
  const float side = t.c_m1 + t.c_p1;
  t.sharp = side > 1e-9f ? std::clamp(1.f - 2.f * t.c_0 / side, 0.f, 1.f) : 0.f;

  // 組合：歧義為硬 gate、曲率為輕度折扣（最多對折）
  const float conf = std::clamp(t.depth * t.unamb * (0.5f + 0.5f * t.sharp), 0.f, 1.f);
  t.result = {static_cast<float>(cost.shift_min + static_cast<int>(t.mi)) + t.delta, conf, true};
  return t;
}

DepthEstimate ParabolicDepthEstimator::estimate(const CostSequence& cost)
{
  return estimateTraced(cost).result;
}

} // namespace pdaf
