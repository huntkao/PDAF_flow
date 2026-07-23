#include "replay/replay_pd_data_source.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace pdaf {
namespace {

using nlohmann::json;

PdInput parseFrame(const json& j, const std::string& file) {
  PdInput in;
  const auto& m = j.at("meta");
  in.meta.frame_id = m.at("frame_id").get<uint64_t>();
  in.meta.timestamp_ms = m.at("timestamp_ms").get<double>();
  in.meta.lens_step_at_exposure = m.at("lens_step_at_exposure").get<int>();

  if (j.contains("hw_costs")) {
    std::vector<CostSequence> costs;
    for (const auto& c : j["hw_costs"]) {
      costs.push_back({c.at("shift_min").get<int>(),
                       c.at("costs").get<std::vector<float>>(),
                       c.at("valid_samples").get<int>()});
    }
    in.hw_costs = std::move(costs);
  } else if (j.contains("rois")) {
    PdFrame f;
    f.meta = in.meta;
    for (const auto& r : j["rois"]) {
      RoiSamples s;
      s.width = r.at("width").get<int>();
      s.height = r.at("height").get<int>();
      s.left = r.at("left").get<std::vector<float>>();
      s.right = r.at("right").get<std::vector<float>>();
      f.rois.push_back(std::move(s));
    }
    in.raw = std::move(f);
  } else {
    throw std::runtime_error("replay: frame has neither 'rois' nor 'hw_costs': " + file);
  }
  return in;
}

}  // namespace

ReplayPdDataSource::ReplayPdDataSource(const std::string& dir) {
  for (int idx = 0;; ++idx) {
    char name[32];
    std::snprintf(name, sizeof(name), "frame_%04d.json", idx);
    std::ifstream f(dir + "/" + name);
    if (!f) break;
    json j;
    try {
      f >> j;
      frames_.push_back(parseFrame(j, name));
    } catch (const json::exception& e) {
      throw std::runtime_error(std::string("replay: bad frame ") + name + ": " + e.what());
    }
  }
  if (frames_.empty())
    throw std::runtime_error("replay: no frame_0000.json found in " + dir);
}

PdInput ReplayPdDataSource::capture(const AfRequest&) {
  const size_t i = std::min(next_, frames_.size() - 1);
  if (next_ + 1 < frames_.size()) ++next_;
  return frames_[i];
}

}  // namespace pdaf
