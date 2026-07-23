#include <gtest/gtest.h>
#include <replay/replay_pd_data_source.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace pdaf;
namespace fs = std::filesystem;

namespace
{

class ReplayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / "pdaf_replay_test";
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override
  {
    fs::remove_all(dir_);
  }
  void writeFrame(int idx, const std::string& body)
  {
    char name[32];
    std::snprintf(name, sizeof(name), "frame_%04d.json", idx);
    std::ofstream(dir_ / name) << body;
  }
  fs::path dir_;
};

} // namespace

TEST_F(ReplayTest, LoadsRawFramesInOrder)
{
  writeFrame(0, R"({"meta":{"frame_id":0,"timestamp_ms":0.0,"lens_step_at_exposure":300},
    "rois":[{"width":4,"height":1,"left":[1,2,3,4],"right":[1,2,3,4]}]})");
  writeFrame(1, R"({"meta":{"frame_id":1,"timestamp_ms":33.3,"lens_step_at_exposure":350},
    "rois":[{"width":4,"height":1,"left":[5,6,7,8],"right":[5,6,7,8]}]})");
  ReplayPdDataSource src(dir_.string());
  EXPECT_EQ(src.frameCount(), 2u);
  auto a = src.capture(AfRequest{});
  ASSERT_TRUE(a.raw.has_value());
  EXPECT_EQ(a.meta.lens_step_at_exposure, 300);
  EXPECT_FLOAT_EQ(a.raw->rois[0].left[0], 1.f);
  auto b = src.capture(AfRequest{});
  EXPECT_EQ(b.meta.frame_id, 1u);
  auto c = src.capture(AfRequest{}); // 放完重複最後一筆
  EXPECT_EQ(c.meta.frame_id, 1u);
}

TEST_F(ReplayTest, LoadsHwCostFrames)
{
  writeFrame(0, R"({"meta":{"frame_id":0,"timestamp_ms":0.0,"lens_step_at_exposure":300},
    "hw_costs":[{"shift_min":-2,"costs":[5,1,5,7,9],"valid_samples":10}]})");
  ReplayPdDataSource src(dir_.string());
  auto a = src.capture(AfRequest{});
  EXPECT_FALSE(a.raw.has_value());
  ASSERT_TRUE(a.hw_costs.has_value());
  EXPECT_EQ((*a.hw_costs)[0].shift_min, -2);
}

TEST_F(ReplayTest, EmptyDirThrows)
{
  EXPECT_THROW(ReplayPdDataSource(dir_.string()), std::runtime_error);
}
