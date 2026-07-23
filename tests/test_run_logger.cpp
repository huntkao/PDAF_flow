#include <gtest/gtest.h>
#include <control/run_logger.h>

#include <filesystem>
#include <fstream>
#include <sstream>

using namespace pdaf;
namespace fs = std::filesystem;

static std::string slurp(const fs::path& p) {
  std::stringstream ss;
  ss << std::ifstream(p).rdbuf();
  return ss.str();
}

TEST(RunLogger, WritesCsvAndSummary) {
  const auto dir = fs::temp_directory_path() / "pdaf_logger_test";
  fs::remove_all(dir);
  {
    RunLogger lg(dir.string());
    AfFrameLog log{7, AfState::kMeasuring, AfState::kMoving, -2.5f, 0.85f, 300, 175};
    lg.logFrame(log, -2.5f);
    lg.writeSummary(AfState::kFocused, 12, 176, 175);
  }
  const auto csv = slurp(dir / "frames.csv");
  EXPECT_NE(csv.find("frame_id,state_before,state_after,disparity,confidence,"
                     "lens_step,target_step,gt_disparity"), std::string::npos);
  EXPECT_NE(csv.find("7,MEASURING,MOVING,-2.5"), std::string::npos);
  const auto sum = slurp(dir / "summary.txt");
  EXPECT_NE(sum.find("final_state: FOCUSED"), std::string::npos);
  EXPECT_NE(sum.find("total_frames: 12"), std::string::npos);
  EXPECT_NE(sum.find("final_step: 176"), std::string::npos);
  EXPECT_NE(sum.find("in_focus_step: 175"), std::string::npos);
  EXPECT_NE(sum.find("final_error_steps: 1"), std::string::npos);
  fs::remove_all(dir);
}
