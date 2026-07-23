#pragma once
#include <pdaf/control/af_controller.h>

#include <fstream>
#include <string>

namespace pdaf
{
// 逐 frame CSV + 收斂摘要（可直接用 Python/gnuplot 繪圖分析）
class RunLogger
{
public:
  explicit RunLogger(const std::string& out_dir);
  void logFrame(const AfFrameLog& log, float gt_disparity = 0.f);
  void writeSummary(AfState final_state, int total_frames, int final_step,
                    int in_focus_step);

private:
  std::string dir_;
  std::ofstream csv_;
};
} // namespace pdaf
