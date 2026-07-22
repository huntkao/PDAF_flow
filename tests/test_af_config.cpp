#include <gtest/gtest.h>
#include <pdaf/control/af_config.h>

#include <fstream>
#include <sstream>

using namespace pdaf;

static std::string readFile(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

TEST(AfConfig, LoadsDefaultJson) {
  auto cfg = AfConfig::loadFromFile(std::string(PDAF_SOURCE_DIR) + "/config/default.json");
  EXPECT_EQ(cfg.sensor.pattern.type, PdPatternDesc::Type::kSparse);
  EXPECT_EQ(cfg.sensor.roi_sample_width, 64);
  EXPECT_NEAR(cfg.calibration.lrc.right_gain, 0.909f, 1e-4f);
  EXPECT_EQ(cfg.calibration.dcc.anchors.size(), 2u);
  EXPECT_EQ(cfg.tuning.shift_min, -16);
  EXPECT_EQ(cfg.system.mode, "sim");
  EXPECT_EQ(cfg.system.sim.settle_frames, 3);
}

TEST(AfConfig, MissingFieldThrowsWithFieldName) {
  auto text = readFile(std::string(PDAF_SOURCE_DIR) + "/config/default.json");
  auto pos = text.find("\"confidence_threshold\": 0.3,");
  ASSERT_NE(pos, std::string::npos);
  text.erase(pos, std::string("\"confidence_threshold\": 0.3,").size());
  try {
    AfConfig::loadFromJson(text);
    FAIL() << "should throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("confidence_threshold"), std::string::npos);
  }
}

TEST(AfConfig, BadModeThrows) {
  auto text = readFile(std::string(PDAF_SOURCE_DIR) + "/config/default.json");
  auto pos = text.find("\"sim\",");
  ASSERT_NE(pos, std::string::npos);
  text.replace(pos, 6, "\"bogus\",");
  EXPECT_THROW(AfConfig::loadFromJson(text), std::runtime_error);
}

TEST(AfConfig, MissingFileThrows) {
  EXPECT_THROW(AfConfig::loadFromFile("/no/such/file.json"), std::runtime_error);
}
