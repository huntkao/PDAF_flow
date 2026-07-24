#include <replay/replay_pd_data_source.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

using namespace pdaf;
namespace fs = std::filesystem;

TEST(CliDump, SimModeWritesParseableCostFrames)
{
  const auto dir = fs::temp_directory_path() / "pdaf_cli_dump_test";
  fs::remove_all(dir);

  const std::string cmd = std::string(PDAF_CLI_BIN) + " --config " + PDAF_SOURCE_DIR +
                          "/config/default.json --out " + (dir / "out").string() +
                          " --dump-costs " + dir.string() + " --max-frames 5";
  const int rc = std::system(cmd.c_str());
  ASSERT_EQ(rc, 0) << cmd;

  ASSERT_TRUE(fs::exists(dir / "frame_0000.json"));
  // replay 能讀回 dump 出來的檔（格式正確、含 hw_costs）
  ReplayPdDataSource src(dir.string());
  auto in = src.capture(AfRequest{});
  EXPECT_TRUE(in.hw_costs.has_value());

  fs::remove_all(dir);
}
