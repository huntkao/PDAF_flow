#include <pdaf/control/af_config.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pdaf {
namespace {

using nlohmann::json;

// fail-fast：缺欄位即丟出含完整路徑的錯誤
const json& req(const json& j, const std::string& key, const std::string& path) {
  auto it = j.find(key);
  if (it == j.end())
    throw std::runtime_error("config: missing field '" + path + "." + key + "'");
  return *it;
}

template <typename T>
T reqAs(const json& j, const std::string& key, const std::string& path) {
  try {
    return req(j, key, path).get<T>();
  } catch (const json::exception&) {
    throw std::runtime_error("config: bad type for '" + path + "." + key + "'");
  }
}

}  // namespace

AfConfig AfConfig::loadFromJson(const std::string& text) {
  json j;
  try {
    j = json::parse(text);
  } catch (const json::exception& e) {
    throw std::runtime_error(std::string("config: invalid JSON: ") + e.what());
  }

  AfConfig c;

  const auto& sensor = req(j, "sensor", "");
  const auto& pat = req(sensor, "pattern", "sensor");
  const auto type = reqAs<std::string>(pat, "type", "sensor.pattern");
  if (type == "sparse") c.sensor.pattern.type = PdPatternDesc::Type::kSparse;
  else if (type == "dual_pd") c.sensor.pattern.type = PdPatternDesc::Type::kDualPd;
  else throw std::runtime_error("config: sensor.pattern.type must be 'sparse' or 'dual_pd'");
  c.sensor.pattern.pair_pitch_x = reqAs<int>(pat, "pair_pitch_x", "sensor.pattern");
  c.sensor.pattern.pair_pitch_y = reqAs<int>(pat, "pair_pitch_y", "sensor.pattern");
  const auto& roi = req(sensor, "default_roi", "sensor");
  c.sensor.default_roi = {reqAs<int>(roi, "x", "sensor.default_roi"),
                          reqAs<int>(roi, "y", "sensor.default_roi"),
                          reqAs<int>(roi, "w", "sensor.default_roi"),
                          reqAs<int>(roi, "h", "sensor.default_roi")};
  c.sensor.roi_sample_width = reqAs<int>(sensor, "roi_sample_width", "sensor");
  c.sensor.roi_sample_height = reqAs<int>(sensor, "roi_sample_height", "sensor");

  const auto& calib = req(j, "calibration", "");
  const auto& lrc = req(calib, "lrc", "calibration");
  c.calibration.lrc.left_gain = reqAs<float>(lrc, "left_gain", "calibration.lrc");
  c.calibration.lrc.right_gain = reqAs<float>(lrc, "right_gain", "calibration.lrc");
  const auto& dcc = req(calib, "dcc", "calibration");
  c.calibration.dcc.step_min = reqAs<int>(dcc, "step_min", "calibration.dcc");
  c.calibration.dcc.step_max = reqAs<int>(dcc, "step_max", "calibration.dcc");
  for (const auto& a : req(dcc, "anchors", "calibration.dcc")) {
    c.calibration.dcc.anchors.push_back(
        {reqAs<int>(a, "step", "calibration.dcc.anchors[]"),
         reqAs<float>(a, "steps_per_disparity", "calibration.dcc.anchors[]")});
  }
  if (c.calibration.dcc.anchors.empty())
    throw std::runtime_error("config: calibration.dcc.anchors must not be empty");

  const auto& tun = req(j, "tuning", "");
  c.tuning.shift_min = reqAs<int>(tun, "shift_min", "tuning");
  c.tuning.shift_max = reqAs<int>(tun, "shift_max", "tuning");
  c.tuning.confidence_threshold = reqAs<float>(tun, "confidence_threshold", "tuning");
  c.tuning.in_focus_disparity = reqAs<float>(tun, "in_focus_disparity", "tuning");
  c.tuning.max_retries = reqAs<int>(tun, "max_retries", "tuning");
  c.tuning.max_iterations = reqAs<int>(tun, "max_iterations", "tuning");

  const auto& sys = req(j, "system", "");
  c.system.mode = reqAs<std::string>(sys, "mode", "system");
  if (c.system.mode != "sim" && c.system.mode != "replay")
    throw std::runtime_error("config: system.mode must be 'sim' or 'replay'");
  c.system.log_dir = reqAs<std::string>(sys, "log_dir", "system");
  c.system.replay_dir = reqAs<std::string>(sys, "replay_dir", "system");
  const auto& sim = req(sys, "sim", "system");
  c.system.sim.object_distance_mm = reqAs<double>(sim, "object_distance_mm", "system.sim");
  c.system.sim.initial_step = reqAs<int>(sim, "initial_step", "system.sim");
  c.system.sim.noise_sigma = reqAs<float>(sim, "noise_sigma", "system.sim");
  c.system.sim.gain_mismatch = reqAs<float>(sim, "gain_mismatch", "system.sim");
  c.system.sim.settle_frames = reqAs<int>(sim, "settle_frames", "system.sim");
  c.system.sim.seed = reqAs<uint32_t>(sim, "seed", "system.sim");
  c.system.sim.step_inf = reqAs<int>(sim, "step_inf", "system.sim");
  c.system.sim.focus_gain = reqAs<float>(sim, "focus_gain", "system.sim");

  return c;
}

AfConfig AfConfig::loadFromFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("config: cannot open file: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return loadFromJson(ss.str());
}

}  // namespace pdaf
