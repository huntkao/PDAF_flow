#include <replay/cost_dump.h>
#include <replay/replay_pd_data_source.h>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

using namespace pdaf;
namespace fs = std::filesystem;

TEST(CostDump, WritesReplayFormatWithGroundTruth)
{
  const auto dir = fs::temp_directory_path() / "pdaf_cost_dump_test";
  fs::remove_all(dir);

  FrameMeta meta{3, 99.9, 300};
  std::vector<CostSequence> costs = {CostSequence{-2, {5, 1, 5, 7, 9}, 40}};
  writeCostFrame(dir.string(), meta, costs, {-2.5f});

  ASSERT_TRUE(fs::exists(dir / "frame_0003.json"));
  EXPECT_FALSE(fs::exists(dir / "frame_0000.json")); // 只寫一個檔，不捏造其他 frame

  std::ifstream f(dir / "frame_0003.json");
  std::stringstream ss;
  ss << f.rdbuf();
  auto j = nlohmann::json::parse(ss.str());
  EXPECT_EQ(j["meta"]["frame_id"].get<uint64_t>(), 3u);
  EXPECT_EQ(j["meta"]["lens_step_at_exposure"].get<int>(), 300);
  EXPECT_EQ(j["hw_costs"][0]["shift_min"].get<int>(), -2);
  EXPECT_EQ(j["hw_costs"][0]["costs"].size(), 5u);
  EXPECT_EQ(j["hw_costs"][0]["valid_samples"].get<int>(), 40);
  EXPECT_FLOAT_EQ(j["ground_truth_disparity"][0].get<float>(), -2.5f);

  fs::remove_all(dir);
}

TEST(CostDump, DumpedFrame0ReadableByReplay)
{
  const auto dir = fs::temp_directory_path() / "pdaf_cost_dump_replay";
  fs::remove_all(dir);
  writeCostFrame(dir.string(), FrameMeta{0, 0.0, 300}, {CostSequence{-2, {5, 1, 5, 7, 9}, 40}}, {-2.5f});
  ReplayPdDataSource src(dir.string());
  auto in = src.capture(AfRequest{});
  ASSERT_TRUE(in.hw_costs.has_value());
  EXPECT_EQ((*in.hw_costs)[0].shift_min, -2);
  fs::remove_all(dir);
}

TEST(CostDump, OmitsGroundTruthWhenEmpty)
{
  const auto dir = fs::temp_directory_path() / "pdaf_cost_dump_test2";
  fs::remove_all(dir);
  writeCostFrame(dir.string(), FrameMeta{0, 0.0, 0}, {CostSequence{0, {1, 0, 1}, 3}}, {});
  std::ifstream f(dir / "frame_0000.json");
  std::stringstream ss;
  ss << f.rdbuf();
  auto j = nlohmann::json::parse(ss.str());
  EXPECT_FALSE(j.contains("ground_truth_disparity"));
  fs::remove_all(dir);
}
