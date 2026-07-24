#include "replay/cost_dump.h"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace pdaf
{

void writeCostFrame(const std::string& dir, const FrameMeta& meta,
                    const std::vector<CostSequence>& costs,
                    const std::vector<float>& ground_truth)
{
  std::filesystem::create_directories(dir);

  nlohmann::json j;
  j["meta"] = {{"frame_id", meta.frame_id},
               {"timestamp_ms", meta.timestamp_ms},
               {"lens_step_at_exposure", meta.lens_step_at_exposure}};
  for (const auto& c : costs)
  {
    j["hw_costs"].push_back({{"shift_min", c.shift_min}, {"costs", c.costs}, {"valid_samples", c.valid_samples}});
  }
  if (!ground_truth.empty())
  {
    j["ground_truth_disparity"] = ground_truth;
  }

  char name[32];
  std::snprintf(name, sizeof(name), "frame_%04llu.json", static_cast<unsigned long long>(meta.frame_id));
  std::ofstream(std::filesystem::path(dir) / name) << j.dump(2) << '\n';
}

} // namespace pdaf
