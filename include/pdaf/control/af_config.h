#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/algo/pd_cost_engine.h>
#include <pdaf/types.h>

#include <cstdint>
#include <string>

namespace pdaf {

struct SensorConfig {
  PdPatternDesc pattern;
  Roi default_roi;
  int roi_sample_width = 64;
  int roi_sample_height = 4;
};

struct CalibrationConfig {
  LrcCalib lrc;
  DccTable dcc;
};

struct TuningConfig {
  int shift_min = -16;
  int shift_max = 16;
  float confidence_threshold = 0.3f;
  float in_focus_disparity = 0.25f;
  int max_retries = 3;
  int max_iterations = 6;
};

struct SimConfig {
  double object_distance_mm = 2000.0;
  int initial_step = 300;
  float noise_sigma = 0.01f;
  float gain_mismatch = 1.1f;
  int settle_frames = 3;
  uint32_t seed = 42;
  int step_inf = 100;        // 無窮遠合焦 step
  float focus_gain = 150000.f;  // in-focus step = step_inf + focus_gain / distance_mm
};

struct SystemConfig {
  std::string mode = "sim";  // "sim" | "replay"
  std::string log_dir = "out";
  std::string replay_dir;
  SimConfig sim;
};

struct AfConfig {
  SensorConfig sensor;
  CalibrationConfig calibration;
  TuningConfig tuning;
  SystemConfig system;

  static AfConfig loadFromFile(const std::string& path);
  static AfConfig loadFromJson(const std::string& text);
};

}  // namespace pdaf
