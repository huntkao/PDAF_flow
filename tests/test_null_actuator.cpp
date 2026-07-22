#include <gtest/gtest.h>
#include <replay/null_lens_actuator.h>

using namespace pdaf;

TEST(NullActuator, RecordsCommandAndNeverMoving) {
  NullLensActuator act(300);
  EXPECT_EQ(act.getStatus().current_step, 300);
  EXPECT_FALSE(act.getStatus().moving);
  act.moveTo(512);
  EXPECT_EQ(act.getStatus().current_step, 512);
  EXPECT_FALSE(act.getStatus().moving);
  ASSERT_EQ(act.history().size(), 1u);
  EXPECT_EQ(act.history()[0], 512);
}
