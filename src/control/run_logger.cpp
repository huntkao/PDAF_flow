#include "control/run_logger.h"

#include <cmath>
#include <filesystem>

namespace pdaf
{

RunLogger::RunLogger(const std::string& out_dir) : dir_(out_dir)
{
  std::filesystem::create_directories(dir_);
  csv_.open(dir_ + "/frames.csv");
  csv_ << "frame_id,state_before,state_after,disparity,confidence,"
          "lens_step,target_step,gt_disparity\n";
}

void RunLogger::logFrame(const AfFrameLog& log, float gt_disparity)
{
  csv_ << log.frame_id << ',' << toString(log.state_before) << ','
       << toString(log.state_after) << ',' << log.disparity << ','
       << log.confidence << ',' << log.lens_step << ',' << log.target_step << ','
       << gt_disparity << '\n';
}

void RunLogger::writeSummary(AfState final_state, int total_frames, int final_step,
                             int in_focus_step)
{
  std::ofstream f(dir_ + "/summary.txt");
  f << "final_state: " << toString(final_state) << '\n'
    << "total_frames: " << total_frames << '\n'
    << "final_step: " << final_step << '\n';
  if (in_focus_step >= 0)
  {
    f << "in_focus_step: " << in_focus_step << '\n'
      << "final_error_steps: " << std::abs(final_step - in_focus_step) << '\n';
  }
}

} // namespace pdaf
