#include "algo/parabolic_depth_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace pdaf
{

DepthEstimate ParabolicDepthEstimator::estimate(const CostSequence& cost)
{
  DepthEstimate e;
  const auto& v = cost.costs;
  if (v.size() < 3 || cost.valid_samples <= 0)
  {
    return e;
  }

  const size_t n = v.size();
  const size_t mi = std::min_element(v.begin(), v.end()) - v.begin();
  const float mean = std::accumulate(v.begin(), v.end(), 0.f) / n;
  if (mean < 1e-6f)
  {
    return e; // 全平：無紋理可匹配
  }
  const float cmin = v[mi];

  // depth：谷底相對整體平均的深度，除以 mean 使其對場景對比尺度不變
  const float depth = std::clamp(1.f - cmin / mean, 0.f, 1.f);

  // 次低點檢查（無歧義度）：先排除主谷「整段單調 basin」——從谷底往兩側走，
  // 只要成本仍在上升就屬於同一個谷；停在成本回降處（被山脊隔開）。這樣寬谷不會把
  // 自己的坡誤判成第二個谷，只有真正分離的競爭谷（如 aliasing 雙峰）才會扣分。
  size_t lo = mi;
  while (lo > 0 && v[lo - 1] >= v[lo])
  {
    --lo;
  }
  size_t hi = mi;
  while (hi + 1 < n && v[hi + 1] >= v[hi])
  {
    ++hi;
  }
  float second = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < n; ++i)
  {
    if (i < lo || i > hi)
    {
      second = std::min(second, v[i]);
    }
  }
  const float depth_abs = mean - cmin;
  const float unamb = (std::isinf(second) || depth_abs < 1e-9f)
                        ? 1.f // 窗太小無可比對象，不因無法判斷而扣分
                        : std::clamp((second - cmin) / depth_abs, 0.f, 1.f);

  if (mi == 0 || mi + 1 == n)
  {
    // 極小值在搜尋邊界：真值可能在範圍外，不內插；曲率未定義，信心對折並套用歧義因子
    e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)), depth * unamb * 0.5f, true};
    return e;
  }
  const float c0 = v[mi - 1], c1 = v[mi], c2 = v[mi + 1];
  const float denom = c0 - 2.f * c1 + c2;
  const float delta = denom > 1e-9f ? 0.5f * (c0 - c2) / denom : 0.f;

  // 曲率／尖銳度：正規化二階差分（= 局部鄰點對比），寬平底 → 接近 0、窄尖谷 → 接近 1
  const float side = c0 + c2;
  const float sharp = side > 1e-9f ? std::clamp(1.f - 2.f * c1 / side, 0.f, 1.f) : 0.f;

  // 組合：歧義為硬 gate（乘 unamb），曲率為輕度折扣（最多對折）——
  // 寬但清楚的谷估值仍正確、只是較不精準，不該打死；有競爭谷才代表可能整個錯
  const float conf = std::clamp(depth * unamb * (0.5f + 0.5f * sharp), 0.f, 1.f);
  e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)) + delta, conf, true};
  return e;
}

} // namespace pdaf
