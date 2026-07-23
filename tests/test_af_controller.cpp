#include <gtest/gtest.h>
#include <pdaf/control/af_controller.h>

#include <deque>

using namespace pdaf;

namespace {

// 可程式化 estimator：依序回吐排好的估測值
class FakeEstimator : public IFocusEstimator {
 public:
  std::deque<DepthEstimate> queue;
  std::vector<DepthEstimate> process(const PdInput&) override {
    if (queue.empty()) return {};
    auto e = queue.front();
    queue.pop_front();
    return {e};
  }
};

// 直通 mapper：target = exposure_step + disparity*50
class FakeMapper : public ILensMapper {
 public:
  int last_base = -1;
  void init(const DccTable&) override {}
  LensCommand toLensCommand(const DepthEstimate& e, int base) override {
    last_base = base;
    return {base + static_cast<int>(e.disparity * 50.f), 5};
  }
};

// 可控 actuator：moving 持續 pending_frames 次查詢
class FakeActuator : public ILensActuator {
 public:
  int step = 300;
  int pending = 0;
  int target = 300;
  void moveTo(int s) override { target = s; pending = 2; }
  LensStatus getStatus() const override { return {step, pending > 0}; }
  void tick() {  // 測試裡每 frame 呼叫，模擬 settle
    if (pending > 0 && --pending == 0) step = target;
  }
};

PdInput frameAt(uint64_t id, int exposure_step) {
  PdInput in;
  in.meta = {id, static_cast<double>(id) * 33.3, exposure_step};
  return in;
}

TuningConfig tun() { return TuningConfig{-16, 16, 0.3f, 0.25f, 2, 4}; }

}  // namespace

TEST(AfController, IdleUntilTriggered) {
  FakeEstimator est;
  FakeMapper map;
  FakeActuator act;
  AfController c(est, map, act, tun());
  EXPECT_EQ(c.state(), AfState::kIdle);
  auto log = c.onFrame(AfRequest{}, frameAt(0, 300));
  EXPECT_EQ(log.state_after, AfState::kIdle);
  c.trigger();
  EXPECT_EQ(c.state(), AfState::kMeasuring);
}

TEST(AfController, HappyPathConvergesToFocused) {
  FakeEstimator est;
  est.queue = {{4.f, 0.9f, true},    // 量測：d=4 → move to 300+200=500
               {0.1f, 0.9f, true}};  // verify：|0.1|<0.25 → Focused
  FakeMapper map;
  FakeActuator act;
  AfController c(est, map, act, tun());
  c.trigger();
  uint64_t id = 0;
  auto log = c.onFrame(AfRequest{}, frameAt(id++, act.step));  // Measuring → Moving
  EXPECT_EQ(log.state_after, AfState::kMoving);
  EXPECT_EQ(log.target_step, 500);
  EXPECT_EQ(map.last_base, 300);  // 用曝光當下位置為基準
  act.tick();
  log = c.onFrame(AfRequest{}, frameAt(id++, act.step));  // Moving → Settling
  EXPECT_EQ(log.state_after, AfState::kSettling);
  act.tick();  // settle 完成，step=500
  log = c.onFrame(AfRequest{}, frameAt(id++, act.step));  // Settling → Verifying
  EXPECT_EQ(log.state_after, AfState::kVerifying);
  log = c.onFrame(AfRequest{}, frameAt(id++, act.step));  // Verifying → Focused
  EXPECT_EQ(log.state_after, AfState::kFocused);
}

TEST(AfController, LowConfidenceRetriesThenFails) {
  FakeEstimator est;
  est.queue = {{0.f, 0.1f, true}, {0.f, 0.1f, true}, {0.f, 0.1f, true}};
  FakeMapper map;
  FakeActuator act;
  AfController c(est, map, act, tun());  // max_retries=2
  c.trigger();
  c.onFrame(AfRequest{}, frameAt(0, 300));
  EXPECT_EQ(c.state(), AfState::kMeasuring);  // retry 1
  c.onFrame(AfRequest{}, frameAt(1, 300));
  EXPECT_EQ(c.state(), AfState::kMeasuring);  // retry 2
  c.onFrame(AfRequest{}, frameAt(2, 300));
  EXPECT_EQ(c.state(), AfState::kFailed);     // 超過 max_retries
}

TEST(AfController, MaxIterationsFails) {
  FakeEstimator est;
  for (int i = 0; i < 10; ++i) est.queue.push_back({4.f, 0.9f, true});  // 永不收斂
  FakeMapper map;
  FakeActuator act;
  AfController c(est, map, act, tun());  // max_iterations=4
  c.trigger();
  for (int i = 0; i < 40 && c.state() != AfState::kFailed; ++i) {
    c.onFrame(AfRequest{}, frameAt(i, act.step));
    act.tick();
  }
  EXPECT_EQ(c.state(), AfState::kFailed);
}

TEST(AfController, RetriggerAfterFocusedResets) {
  FakeEstimator est;
  est.queue = {{0.1f, 0.9f, true}};  // 一開始就合焦
  FakeMapper map;
  FakeActuator act;
  AfController c(est, map, act, tun());
  c.trigger();
  c.onFrame(AfRequest{}, frameAt(0, 300));
  EXPECT_EQ(c.state(), AfState::kFocused);
  c.trigger();
  EXPECT_EQ(c.state(), AfState::kMeasuring);
}

TEST(AfController, LensCommandUsesExposureStepNotActuatorPosition) {
  FakeEstimator est;
  est.queue = {{4.f, 0.9f, true}};  // valid, high-confidence, out of focus -> triggers a move
  FakeMapper map;
  FakeActuator act;              // act.step == 300 (default)
  AfController c(est, map, act, tun());
  c.trigger();
  // Feed a measurement frame whose exposure-time lens position (250) differs
  // from the actuator's current step (300). If the controller wrongly used the
  // actuator position, last_base would be 300.
  c.onFrame(AfRequest{}, frameAt(0, 250));
  EXPECT_EQ(map.last_base, 250);  // exposure step, NOT actuator's 300
}
