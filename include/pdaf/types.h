#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace pdaf
{

struct PdPatternDesc
{
  enum class Type
  {
    kSparse,
    kDualPd
  };
  Type type = Type::kSparse;
  int pair_pitch_x = 16; // 相鄰 L/R pair 的像素間距
  int pair_pitch_y = 16;
};

struct Roi
{
  int x = 0, y = 0, w = 0, h = 0;
};

// 每 frame 進 onFrame() 的請求：動態 ROI（touch AF、人臉框）
struct AfRequest
{
  std::vector<Roi> rois;
};

// 實機 stats 有 pipeline 延遲：lens 位置必須以曝光當下為準
struct FrameMeta
{
  uint64_t frame_id = 0;
  double timestamp_ms = 0.0;
  int lens_step_at_exposure = 0;
};

struct RoiSamples
{
  int width = 0;           // 每列 sample 數
  int height = 0;          // 列數
  std::vector<float> left; // row-major，大小 width*height
  std::vector<float> right;
};

struct PdFrame
{
  FrameMeta meta;
  PdPatternDesc pattern;
  std::vector<RoiSamples> rois; // 與 AfRequest::rois 依序對齊
};

struct CostSequence
{
  int shift_min = 0;
  std::vector<float> costs; // costs[i] 對應 shift = shift_min + i
  int valid_samples = 0;
};

// HAL 資料來源輸出（tagged variant）：raw 或 ISP HW 已算好的 cost
struct PdInput
{
  FrameMeta meta;
  std::optional<PdFrame> raw;
  std::optional<std::vector<CostSequence>> hw_costs;
};

struct DepthEstimate
{
  float disparity = 0.f;  // 單位：PD sample shift（sub-pixel）
  float confidence = 0.f; // 0~1
  bool valid = false;
};

struct LensCommand
{
  int target_step = 0;
  int tolerance = 0;
};
struct LensStatus
{
  int current_step = 0;
  bool moving = false;
};

} // namespace pdaf
