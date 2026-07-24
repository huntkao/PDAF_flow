#pragma once
#include <pdaf/types.h>

#include <string>
#include <vector>

namespace pdaf
{
// 把一個 frame 的 cost sequences 寫成 replay hw_costs 格式 JSON（<dir>/frame_%04d.json）。
// ground_truth 非空時附上 ground_truth_disparity（size 需等於 costs.size()）。
void writeCostFrame(const std::string& dir, const FrameMeta& meta,
                    const std::vector<CostSequence>& costs,
                    const std::vector<float>& ground_truth);
} // namespace pdaf
