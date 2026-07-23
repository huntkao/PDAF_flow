# PDAF_flow 框架實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立 PDAF 自動對焦 demo 框架：三個演算法模塊（介面+參考實作）、HAL 抽象、單次 AF 狀態機、光學模擬器閉環、dump 重播、CLI。

**Architecture:** 四層（HAL / 演算法 / 控制 / 應用），核心編成 `libpdaf` static library。`AfController` 只依賴四個介面（`IPdDataSource`、`ILensActuator`、`IFocusEstimator`、`ILensMapper`）。詳見 spec：`docs/superpowers/specs/2026-07-22-pdaf-flow-design.md`。

**Tech Stack:** C++17、CMake ≥3.16、GoogleTest（FetchContent v1.14.0）、nlohmann/json（vendored 單頭檔 v3.11.3）。

## Global Constraints

- C++17，不依賴 OpenCV；模擬器運算自行實作
- 所有程式碼在 `namespace pdaf`
- 執行期演算法異常不丟例外，反映為 `confidence=0`/`valid=false`；config 錯誤 fail-fast 丟 `std::runtime_error` 並指出欄位
- 測試指令一律：`cmake --build build && ctest --test-dir build --output-on-failure`（單一測試 binary `pdaf_tests`，可用 `build/tests/pdaf_tests --gtest_filter=<Suite>.*` 跑單一 suite）
- 每個 task 結尾 commit，訊息用 conventional commits（feat/test/build/docs）

---

### Task 1: 專案骨架（CMake + GoogleTest + vendored json）

**Files:**
- Create: `CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/test_smoke.cpp`, `third_party/nlohmann/json.hpp`（下載）, `.gitignore`
- Create: `include/pdaf/types.h`（本 task 一併建立核心資料型別）

**Interfaces:**
- Produces: `libpdaf` target（include path：`include/`、`third_party/`）；`pdaf::` 核心型別（下方 types.h 全文，後續所有 task 依賴）

- [ ] **Step 1: 建立 .gitignore 與下載 json.hpp**

```gitignore
build/
out/
```

```bash
mkdir -p third_party/nlohmann
curl -L -o third_party/nlohmann/json.hpp https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp
```

- [ ] **Step 2: 建立 `include/pdaf/types.h`**

```cpp
#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace pdaf {

struct PdPatternDesc {
  enum class Type { kSparse, kDualPd };
  Type type = Type::kSparse;
  int pair_pitch_x = 16;  // 相鄰 L/R pair 的像素間距
  int pair_pitch_y = 16;
};

struct Roi { int x = 0, y = 0, w = 0, h = 0; };

// 每 frame 進 onFrame() 的請求：動態 ROI（touch AF、人臉框）
struct AfRequest { std::vector<Roi> rois; };

// 實機 stats 有 pipeline 延遲：lens 位置必須以曝光當下為準
struct FrameMeta {
  uint64_t frame_id = 0;
  double timestamp_ms = 0.0;
  int lens_step_at_exposure = 0;
};

struct RoiSamples {
  int width = 0;   // 每列 sample 數
  int height = 0;  // 列數
  std::vector<float> left;   // row-major，大小 width*height
  std::vector<float> right;
};

struct PdFrame {
  FrameMeta meta;
  PdPatternDesc pattern;
  std::vector<RoiSamples> rois;  // 與 AfRequest::rois 依序對齊
};

struct CostSequence {
  int shift_min = 0;
  std::vector<float> costs;  // costs[i] 對應 shift = shift_min + i
  int valid_samples = 0;
};

// HAL 資料來源輸出（tagged variant）：raw 或 ISP HW 已算好的 cost
struct PdInput {
  FrameMeta meta;
  std::optional<PdFrame> raw;
  std::optional<std::vector<CostSequence>> hw_costs;
};

struct DepthEstimate {
  float disparity = 0.f;   // 單位：PD sample shift（sub-pixel）
  float confidence = 0.f;  // 0~1
  bool valid = false;
};

struct LensCommand { int target_step = 0; int tolerance = 0; };
struct LensStatus { int current_step = 0; bool moving = false; };

}  // namespace pdaf
```

- [ ] **Step 3: 建立根 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(pdaf_flow CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(pdaf STATIC)
target_include_directories(pdaf PUBLIC include third_party)
# 先給一個空的翻譯單元讓 static lib 可建；後續 task 逐步加檔
file(WRITE ${CMAKE_BINARY_DIR}/pdaf_dummy.cpp "namespace pdaf { int _dummy = 0; }\n")
target_sources(pdaf PRIVATE ${CMAKE_BINARY_DIR}/pdaf_dummy.cpp)

enable_testing()
add_subdirectory(tests)
```

- [ ] **Step 4: 建立 `tests/CMakeLists.txt` 與 smoke test**

```cmake
include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(pdaf_tests test_smoke.cpp)
target_link_libraries(pdaf_tests PRIVATE pdaf GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(pdaf_tests)
```

`tests/test_smoke.cpp`：

```cpp
#include <gtest/gtest.h>
#include <pdaf/types.h>

TEST(Smoke, TypesCompileAndDefault) {
  pdaf::PdInput in;
  EXPECT_FALSE(in.raw.has_value());
  EXPECT_FALSE(in.hw_costs.has_value());
  pdaf::DepthEstimate e;
  EXPECT_FALSE(e.valid);
}
```

- [ ] **Step 5: 建置並跑測試**

Run: `cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `Smoke.TypesCompileAndDefault ... Passed`，`100% tests passed`

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tests/ include/ third_party/ .gitignore
git commit -m "build: 專案骨架（CMake+GoogleTest+vendored json）與核心資料型別"
```

---

### Task 2: M2 參考實作 — ParabolicDepthEstimator

**Files:**
- Create: `include/pdaf/algo/depth_estimator.h`, `src/algo/parabolic_depth_estimator.h`, `src/algo/parabolic_depth_estimator.cpp`
- Test: `tests/test_depth_estimator.cpp`
- Modify: 根 `CMakeLists.txt`（`target_sources` 加 `src/algo/parabolic_depth_estimator.cpp`；並加 `target_include_directories(pdaf PUBLIC src)`——src 下的實作標頭供測試與組裝取用）、`tests/CMakeLists.txt`（`add_executable` 加 `test_depth_estimator.cpp`）

**Interfaces:**
- Consumes: `pdaf::CostSequence`, `pdaf::DepthEstimate`（Task 1）
- Produces: `class IDepthEstimator { virtual DepthEstimate estimate(const CostSequence&) = 0; }`；`class ParabolicDepthEstimator : public IDepthEstimator`

- [ ] **Step 1: 寫失敗測試 `tests/test_depth_estimator.cpp`**

```cpp
#include <gtest/gtest.h>
#include <algo/parabolic_depth_estimator.h>

using namespace pdaf;

static CostSequence makeCost(int shift_min, std::vector<float> costs, int valid = 100) {
  return CostSequence{shift_min, std::move(costs), valid};
}

TEST(DepthEstimator, FindsIntegerMinimum) {
  // 極小值在 shift=+2（對稱 → 無 sub-pixel 偏移）
  auto c = makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4});
  auto e = ParabolicDepthEstimator{}.estimate(c);
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 1.0f, 0.01f);  // min at index 5 → shift -4+5=+1
  EXPECT_GT(e.confidence, 0.5f);
}

TEST(DepthEstimator, SubPixelInterpolation) {
  // 拋物線 (s-0.5)^2 取樣於整數 shift → 內插應得 0.5
  std::vector<float> v;
  for (int s = -4; s <= 4; ++s) v.push_back((s - 0.5f) * (s - 0.5f));
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, v));
  EXPECT_TRUE(e.valid);
  EXPECT_NEAR(e.disparity, 0.5f, 0.01f);
}

TEST(DepthEstimator, FlatCurveGivesZeroConfidence) {
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, std::vector<float>(9, 5.f)));
  EXPECT_LT(e.confidence, 0.1f);
}

TEST(DepthEstimator, BoundaryMinimumHalvesConfidence) {
  auto edge = ParabolicDepthEstimator{}.estimate(makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}));
  auto mid  = ParabolicDepthEstimator{}.estimate(makeCost(-4, {2, 1, 0.2f, 1, 2, 4, 6, 8, 10}));
  EXPECT_TRUE(edge.valid);
  EXPECT_NEAR(edge.disparity, -4.f, 0.01f);  // 邊界不內插
  EXPECT_LT(edge.confidence, mid.confidence);
}

TEST(DepthEstimator, NoValidSamplesIsInvalid) {
  auto e = ParabolicDepthEstimator{}.estimate(makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0));
  EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.confidence, 0.f);
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `algo/parabolic_depth_estimator.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/algo/depth_estimator.h`：

```cpp
#pragma once
#include <pdaf/types.h>

namespace pdaf {
// M2：由 cost sequence 推估 disparity 與 confidence
class IDepthEstimator {
 public:
  virtual ~IDepthEstimator() = default;
  virtual DepthEstimate estimate(const CostSequence& cost) = 0;
};
}  // namespace pdaf
```

`src/algo/parabolic_depth_estimator.h`：

```cpp
#pragma once
#include <pdaf/algo/depth_estimator.h>

namespace pdaf {
// 參考實作：cost 極小值 + 三點拋物線內插；confidence 由曲線相對深度導出
class ParabolicDepthEstimator : public IDepthEstimator {
 public:
  DepthEstimate estimate(const CostSequence& cost) override;
};
}  // namespace pdaf
```

`src/algo/parabolic_depth_estimator.cpp`：

```cpp
#include "algo/parabolic_depth_estimator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace pdaf {

DepthEstimate ParabolicDepthEstimator::estimate(const CostSequence& cost) {
  DepthEstimate e;
  const auto& v = cost.costs;
  if (v.size() < 3 || cost.valid_samples <= 0) return e;

  const size_t mi = std::min_element(v.begin(), v.end()) - v.begin();
  const float mean = std::accumulate(v.begin(), v.end(), 0.f) / v.size();
  if (mean < 1e-6f) return e;  // 全平：無紋理可匹配

  const float conf = std::clamp(1.f - v[mi] / mean, 0.f, 1.f);
  if (mi == 0 || mi + 1 == v.size()) {
    // 極小值在搜尋邊界：真值可能在範圍外，不內插且信心減半
    e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)), conf * 0.5f, true};
    return e;
  }
  const float c0 = v[mi - 1], c1 = v[mi], c2 = v[mi + 1];
  const float denom = c0 - 2.f * c1 + c2;
  const float delta = denom > 1e-9f ? 0.5f * (c0 - c2) / denom : 0.f;
  e = {static_cast<float>(cost.shift_min + static_cast<int>(mi)) + delta, conf, true};
  return e;
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 DepthEstimator 5 條）

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/algo/ src/algo/ tests/ CMakeLists.txt
git commit -m "feat: M2 參考實作 ParabolicDepthEstimator（拋物線內插+confidence）"
```

---

### Task 3: M1 參考實作 — SadCostEngine

**Files:**
- Create: `include/pdaf/algo/pd_cost_engine.h`, `src/algo/sad_cost_engine.h`, `src/algo/sad_cost_engine.cpp`
- Test: `tests/test_cost_engine.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/algo/sad_cost_engine.cpp`）、`tests/CMakeLists.txt`（加 `test_cost_engine.cpp`）

**Interfaces:**
- Consumes: `PdFrame`, `CostSequence`, `PdPatternDesc`（Task 1）
- Produces: `struct LrcCalib { float left_gain = 1.f; float right_gain = 1.f; }`；`class IPdCostEngine { virtual void init(const LrcCalib&, const PdPatternDesc&, int shift_min, int shift_max); virtual std::vector<CostSequence> compute(const PdFrame&); }`；`class SadCostEngine : public IPdCostEngine`

- [ ] **Step 1: 寫失敗測試 `tests/test_cost_engine.cpp`**

```cpp
#include <gtest/gtest.h>
#include <algo/sad_cost_engine.h>

#include <algorithm>
#include <cmath>

using namespace pdaf;

// 產生 L=tex(i)、R=tex(i-d)*gain 的單 ROI frame
static PdFrame makeShiftedFrame(int d, float right_gain_err, int n = 64) {
  PdFrame f;
  RoiSamples r;
  r.width = n; r.height = 1;
  auto tex = [](float x) { return 2.f + std::sin(0.37f * x) + 0.5f * std::sin(1.13f * x); };
  for (int i = 0; i < n; ++i) {
    r.left.push_back(tex(static_cast<float>(i)));
    r.right.push_back(tex(static_cast<float>(i - d)) * right_gain_err);
  }
  f.rois.push_back(std::move(r));
  return f;
}

static int argmin(const std::vector<float>& v) {
  return static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin());
}

TEST(CostEngine, MinimumAtTrueShift) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -8, 8);
  auto out = m1.compute(makeShiftedFrame(3, 1.f));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].shift_min, -8);
  EXPECT_EQ(out[0].costs.size(), 17u);
  EXPECT_EQ(out[0].shift_min + argmin(out[0].costs), 3);
  EXPECT_GT(out[0].valid_samples, 0);
}

TEST(CostEngine, LrcCorrectionCancelsGainMismatch) {
  // R 通道 gain 高 20%；校正 right_gain=1/1.2 後極小值應回到真 shift 且 cost 近 0
  SadCostEngine m1;
  m1.init(LrcCalib{1.f, 1.f / 1.2f}, PdPatternDesc{}, -8, 8);
  auto out = m1.compute(makeShiftedFrame(-2, 1.2f));
  EXPECT_EQ(out[0].shift_min + argmin(out[0].costs), -2);
  EXPECT_LT(*std::min_element(out[0].costs.begin(), out[0].costs.end()), 0.05f);
}

TEST(CostEngine, MultiRoiProducesOneSequenceEach) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -4, 4);
  auto f = makeShiftedFrame(1, 1.f);
  f.rois.push_back(f.rois[0]);
  auto out = m1.compute(f);
  EXPECT_EQ(out.size(), 2u);
}

TEST(CostEngine, EmptyRoiGivesZeroValidSamples) {
  SadCostEngine m1;
  m1.init(LrcCalib{}, PdPatternDesc{}, -4, 4);
  PdFrame f;
  f.rois.push_back(RoiSamples{});  // width=height=0
  auto out = m1.compute(f);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].valid_samples, 0);  // 不丟例外，降級
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `algo/sad_cost_engine.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/algo/pd_cost_engine.h`：

```cpp
#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf {

// LRC/SPC 校正資料（參考實作用全域 gain；正式版可擴充為 per-position 表）
struct LrcCalib {
  float left_gain = 1.f;
  float right_gain = 1.f;
};

// M1：套用 LRC/SPC 校正，計算各 ROI 的 matching cost sequence
class IPdCostEngine {
 public:
  virtual ~IPdCostEngine() = default;
  virtual void init(const LrcCalib& calib, const PdPatternDesc& pattern,
                    int shift_min, int shift_max) = 0;
  virtual std::vector<CostSequence> compute(const PdFrame& frame) = 0;
};

}  // namespace pdaf
```

`src/algo/sad_cost_engine.h`：

```cpp
#pragma once
#include <pdaf/algo/pd_cost_engine.h>

namespace pdaf {
// 參考實作：gain 校正後逐 shift 計算 mean SAD
class SadCostEngine : public IPdCostEngine {
 public:
  void init(const LrcCalib& calib, const PdPatternDesc& pattern,
            int shift_min, int shift_max) override;
  std::vector<CostSequence> compute(const PdFrame& frame) override;

 private:
  LrcCalib calib_;
  int shift_min_ = -8;
  int shift_max_ = 8;
};
}  // namespace pdaf
```

`src/algo/sad_cost_engine.cpp`：

```cpp
#include "algo/sad_cost_engine.h"

#include <cmath>

namespace pdaf {

void SadCostEngine::init(const LrcCalib& calib, const PdPatternDesc&,
                         int shift_min, int shift_max) {
  calib_ = calib;
  shift_min_ = shift_min;
  shift_max_ = shift_max;
}

std::vector<CostSequence> SadCostEngine::compute(const PdFrame& frame) {
  std::vector<CostSequence> out;
  for (const auto& r : frame.rois) {
    CostSequence cs;
    cs.shift_min = shift_min_;
    const int n = r.width;
    for (int s = shift_min_; s <= shift_max_; ++s) {
      double acc = 0.0;
      int cnt = 0;
      for (int y = 0; y < r.height; ++y) {
        for (int i = 0; i < n; ++i) {
          const int j = i + s;
          if (j < 0 || j >= n) continue;
          const float l = r.left[y * n + i] * calib_.left_gain;
          const float rr = r.right[y * n + j] * calib_.right_gain;
          acc += std::abs(l - rr);
          ++cnt;
        }
      }
      cs.costs.push_back(cnt > 0 ? static_cast<float>(acc / cnt) : 0.f);
    }
    cs.valid_samples = n * r.height;
    out.push_back(std::move(cs));
  }
  return out;
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 CostEngine 4 條）

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/algo/pd_cost_engine.h src/algo/sad_cost_engine.* tests/ CMakeLists.txt
git commit -m "feat: M1 參考實作 SadCostEngine（LRC gain 校正+SAD cost）"
```

---

### Task 4: M3 參考實作 — DccLensMapper

**Files:**
- Create: `include/pdaf/algo/lens_mapper.h`, `src/algo/dcc_lens_mapper.h`, `src/algo/dcc_lens_mapper.cpp`
- Test: `tests/test_lens_mapper.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/algo/dcc_lens_mapper.cpp`）、`tests/CMakeLists.txt`（加 `test_lens_mapper.cpp`）

**Interfaces:**
- Consumes: `DepthEstimate`, `LensCommand`（Task 1）
- Produces: `struct DccAnchor { int step; float steps_per_disparity; }`；`struct DccTable { int step_min = 0; int step_max = 1023; std::vector<DccAnchor> anchors; }`；`float dccInterp(const DccTable&, int step)`（自由函式，模擬器 Task 8 也用它保持閉環一致）；`class ILensMapper { virtual void init(const DccTable&); virtual LensCommand toLensCommand(const DepthEstimate&, int lens_step_at_exposure); }`；`class DccLensMapper : public ILensMapper`

- [ ] **Step 1: 寫失敗測試 `tests/test_lens_mapper.cpp`**

```cpp
#include <gtest/gtest.h>
#include <algo/dcc_lens_mapper.h>

using namespace pdaf;

static DccTable twoAnchor() {
  return DccTable{0, 1023, {{0, 40.f}, {1000, 60.f}}};
}

TEST(LensMapper, DccInterpolatesBetweenAnchors) {
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 500), 50.f);
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 0), 40.f);
  EXPECT_FLOAT_EQ(dccInterp(twoAnchor(), 1023), 60.f);  // 超出末錨點 → clamp
}

TEST(LensMapper, TargetUsesExposureStepAsBase) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  // 曝光當下 step=500（dcc=50），disparity=+2 → 500 + 2*50 = 600
  auto cmd = m3.toLensCommand(DepthEstimate{2.f, 0.9f, true}, 500);
  EXPECT_EQ(cmd.target_step, 600);
}

TEST(LensMapper, TargetClampedToStepRange) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  auto cmd = m3.toLensCommand(DepthEstimate{-100.f, 0.9f, true}, 100);
  EXPECT_EQ(cmd.target_step, 0);
  cmd = m3.toLensCommand(DepthEstimate{100.f, 0.9f, true}, 900);
  EXPECT_EQ(cmd.target_step, 1023);
}

TEST(LensMapper, ToleranceScalesWithDcc) {
  DccLensMapper m3;
  m3.init(twoAnchor());
  auto cmd = m3.toLensCommand(DepthEstimate{1.f, 0.9f, true}, 500);
  EXPECT_GT(cmd.tolerance, 0);  // 約 0.25 disparity 對應的 step 數
  EXPECT_LE(cmd.tolerance, 20);
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `algo/dcc_lens_mapper.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/algo/lens_mapper.h`：

```cpp
#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf {

struct DccAnchor {
  int step = 0;
  float steps_per_disparity = 50.f;  // DCC：1 單位 disparity 對應的 VCM step 數
};

struct DccTable {
  int step_min = 0;
  int step_max = 1023;
  std::vector<DccAnchor> anchors;  // 依 step 遞增排序，至少 1 個
};

// 錨點間線性內插、兩端 clamp（模擬器共用，確保閉環一致）
float dccInterp(const DccTable& table, int step);

// M3：套用 DCC 校正，輸出 VCM step 命令；基準為曝光當下的 lens 位置
class ILensMapper {
 public:
  virtual ~ILensMapper() = default;
  virtual void init(const DccTable& table) = 0;
  virtual LensCommand toLensCommand(const DepthEstimate& est,
                                    int lens_step_at_exposure) = 0;
};

}  // namespace pdaf
```

`src/algo/dcc_lens_mapper.h`：

```cpp
#pragma once
#include <pdaf/algo/lens_mapper.h>

namespace pdaf {
class DccLensMapper : public ILensMapper {
 public:
  void init(const DccTable& table) override;
  LensCommand toLensCommand(const DepthEstimate& est,
                            int lens_step_at_exposure) override;

 private:
  DccTable table_;
};
}  // namespace pdaf
```

`src/algo/dcc_lens_mapper.cpp`：

```cpp
#include "algo/dcc_lens_mapper.h"

#include <algorithm>
#include <cmath>

namespace pdaf {

float dccInterp(const DccTable& table, int step) {
  const auto& a = table.anchors;
  if (a.empty()) return 50.f;
  if (step <= a.front().step) return a.front().steps_per_disparity;
  if (step >= a.back().step) return a.back().steps_per_disparity;
  for (size_t i = 1; i < a.size(); ++i) {
    if (step <= a[i].step) {
      const float t = static_cast<float>(step - a[i - 1].step) /
                      static_cast<float>(a[i].step - a[i - 1].step);
      return a[i - 1].steps_per_disparity +
             t * (a[i].steps_per_disparity - a[i - 1].steps_per_disparity);
    }
  }
  return a.back().steps_per_disparity;
}

void DccLensMapper::init(const DccTable& table) { table_ = table; }

LensCommand DccLensMapper::toLensCommand(const DepthEstimate& est,
                                         int lens_step_at_exposure) {
  const float k = dccInterp(table_, lens_step_at_exposure);
  int target = static_cast<int>(std::lround(lens_step_at_exposure + est.disparity * k));
  target = std::clamp(target, table_.step_min, table_.step_max);
  const int tol = static_cast<int>(std::ceil(std::abs(k) * 0.25f));
  return {target, tol};
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 LensMapper 4 條）

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/algo/lens_mapper.h src/algo/dcc_lens_mapper.* tests/ CMakeLists.txt
git commit -m "feat: M3 參考實作 DccLensMapper（DCC 錨點內插+step clamp）"
```

---

### Task 5: HAL 介面 + NullLensActuator

**Files:**
- Create: `include/pdaf/hal/pd_data_source.h`, `include/pdaf/hal/lens_actuator.h`, `src/replay/null_lens_actuator.h`
- Test: `tests/test_null_actuator.cpp`
- Modify: `tests/CMakeLists.txt`（加 `test_null_actuator.cpp`）

**Interfaces:**
- Consumes: `PdInput`, `AfRequest`, `LensStatus`（Task 1）
- Produces: `class IPdDataSource { virtual PdInput capture(const AfRequest&) = 0; }`；`class ILensActuator { virtual void moveTo(int step) = 0; virtual LensStatus getStatus() const = 0; }`；`class NullLensActuator : public ILensActuator`（記錄命令、瞬移、不 moving——replay 模式用）

- [ ] **Step 1: 寫失敗測試 `tests/test_null_actuator.cpp`**

```cpp
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
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `replay/null_lens_actuator.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/hal/pd_data_source.h`：

```cpp
#pragma once
#include <pdaf/types.h>

namespace pdaf {
// HAL：phase pixel 資料來源（模擬器 / dump 重播 / 真硬體）
class IPdDataSource {
 public:
  virtual ~IPdDataSource() = default;
  virtual PdInput capture(const AfRequest& request) = 0;
};
}  // namespace pdaf
```

`include/pdaf/hal/lens_actuator.h`：

```cpp
#pragma once
#include <pdaf/types.h>

namespace pdaf {
// HAL：VCM actuator
class ILensActuator {
 public:
  virtual ~ILensActuator() = default;
  virtual void moveTo(int step) = 0;
  virtual LensStatus getStatus() const = 0;
};
}  // namespace pdaf
```

`src/replay/null_lens_actuator.h`：

```cpp
#pragma once
#include <pdaf/hal/lens_actuator.h>

#include <vector>

namespace pdaf {
// replay 模式：只記錄命令，瞬移、永不 moving（不影響重播資料）
class NullLensActuator : public ILensActuator {
 public:
  explicit NullLensActuator(int initial_step = 0) : step_(initial_step) {}
  void moveTo(int step) override {
    step_ = step;
    history_.push_back(step);
  }
  LensStatus getStatus() const override { return {step_, false}; }
  const std::vector<int>& history() const { return history_; }

 private:
  int step_;
  std::vector<int> history_;
};
}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/hal/ src/replay/ tests/
git commit -m "feat: HAL 介面（IPdDataSource/ILensActuator）與 NullLensActuator"
```

---

### Task 6: Config 系統（AfConfig JSON 載入 + fail-fast）

**Files:**
- Create: `include/pdaf/control/af_config.h`, `src/control/af_config.cpp`, `config/default.json`
- Test: `tests/test_af_config.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/control/af_config.cpp`）、`tests/CMakeLists.txt`（加 `test_af_config.cpp`）

**Interfaces:**
- Consumes: `LrcCalib`（Task 3）、`DccTable`/`DccAnchor`（Task 4）、`PdPatternDesc`/`Roi`（Task 1）
- Produces:

```cpp
struct SensorConfig { PdPatternDesc pattern; Roi default_roi; int roi_sample_width; int roi_sample_height; };
struct CalibrationConfig { LrcCalib lrc; DccTable dcc; };
struct TuningConfig { int shift_min; int shift_max; float confidence_threshold;
                      float in_focus_disparity; int max_retries; int max_iterations; };
struct SimConfig { double object_distance_mm; int initial_step; float noise_sigma;
                   float gain_mismatch; int settle_frames; uint32_t seed;
                   int step_inf; float focus_gain; };
struct SystemConfig { std::string mode; /* "sim" | "replay" */ std::string log_dir;
                      std::string replay_dir; SimConfig sim; };
struct AfConfig { SensorConfig sensor; CalibrationConfig calibration;
                  TuningConfig tuning; SystemConfig system;
                  static AfConfig loadFromFile(const std::string& path);   // 檔案不存在/JSON 壞 → throw
                  static AfConfig loadFromJson(const std::string& text); };  // 缺欄位 → throw 指出欄位
```

- [ ] **Step 1: 建立 `config/default.json`**

```json
{
  "sensor": {
    "pattern": { "type": "sparse", "pair_pitch_x": 16, "pair_pitch_y": 16 },
    "default_roi": { "x": 960, "y": 540, "w": 256, "h": 128 },
    "roi_sample_width": 64,
    "roi_sample_height": 4
  },
  "calibration": {
    "lrc": { "left_gain": 1.0, "right_gain": 0.909 },
    "dcc": {
      "step_min": 0,
      "step_max": 1023,
      "anchors": [
        { "step": 0, "steps_per_disparity": 50.0 },
        { "step": 1023, "steps_per_disparity": 50.0 }
      ]
    }
  },
  "tuning": {
    "shift_min": -16,
    "shift_max": 16,
    "confidence_threshold": 0.3,
    "in_focus_disparity": 0.25,
    "max_retries": 3,
    "max_iterations": 6
  },
  "system": {
    "mode": "sim",
    "log_dir": "out",
    "replay_dir": "",
    "sim": {
      "object_distance_mm": 2000.0,
      "initial_step": 300,
      "noise_sigma": 0.01,
      "gain_mismatch": 1.1,
      "settle_frames": 3,
      "seed": 42,
      "step_inf": 100,
      "focus_gain": 150000.0
    }
  }
}
```

（`lrc.right_gain = 0.909 ≈ 1/1.1`，剛好抵銷模擬器的 `gain_mismatch = 1.1`——校正資料與「機台特性」對應，正是實機 SPC/LRC 的關係。）

- [ ] **Step 2: 寫失敗測試 `tests/test_af_config.cpp`**

```cpp
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
```

並在 `tests/CMakeLists.txt` 的 `target_link_libraries` 後加：

```cmake
target_compile_definitions(pdaf_tests PRIVATE PDAF_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
```

- [ ] **Step 3: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `pdaf/control/af_config.h: No such file or directory`

- [ ] **Step 4: 實作**

`include/pdaf/control/af_config.h`：

```cpp
#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/algo/pd_cost_engine.h>
#include <pdaf/types.h>

#include <cstdint>
#include <string>

namespace pdaf {

struct SensorConfig {
  PdPatternDesc pattern;
  Roi default_roi;
  int roi_sample_width = 64;
  int roi_sample_height = 4;
};

struct CalibrationConfig {
  LrcCalib lrc;
  DccTable dcc;
};

struct TuningConfig {
  int shift_min = -16;
  int shift_max = 16;
  float confidence_threshold = 0.3f;
  float in_focus_disparity = 0.25f;
  int max_retries = 3;
  int max_iterations = 6;
};

struct SimConfig {
  double object_distance_mm = 2000.0;
  int initial_step = 300;
  float noise_sigma = 0.01f;
  float gain_mismatch = 1.1f;
  int settle_frames = 3;
  uint32_t seed = 42;
  int step_inf = 100;        // 無窮遠合焦 step
  float focus_gain = 150000.f;  // in-focus step = step_inf + focus_gain / distance_mm
};

struct SystemConfig {
  std::string mode = "sim";  // "sim" | "replay"
  std::string log_dir = "out";
  std::string replay_dir;
  SimConfig sim;
};

struct AfConfig {
  SensorConfig sensor;
  CalibrationConfig calibration;
  TuningConfig tuning;
  SystemConfig system;

  static AfConfig loadFromFile(const std::string& path);
  static AfConfig loadFromJson(const std::string& text);
};

}  // namespace pdaf
```

`src/control/af_config.cpp`：

```cpp
#include <pdaf/control/af_config.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pdaf {
namespace {

using nlohmann::json;

// fail-fast：缺欄位即丟出含完整路徑的錯誤
const json& req(const json& j, const std::string& key, const std::string& path) {
  auto it = j.find(key);
  if (it == j.end())
    throw std::runtime_error("config: missing field '" + path + "." + key + "'");
  return *it;
}

template <typename T>
T reqAs(const json& j, const std::string& key, const std::string& path) {
  try {
    return req(j, key, path).get<T>();
  } catch (const json::exception&) {
    throw std::runtime_error("config: bad type for '" + path + "." + key + "'");
  }
}

}  // namespace

AfConfig AfConfig::loadFromJson(const std::string& text) {
  json j;
  try {
    j = json::parse(text);
  } catch (const json::exception& e) {
    throw std::runtime_error(std::string("config: invalid JSON: ") + e.what());
  }

  AfConfig c;

  const auto& sensor = req(j, "sensor", "");
  const auto& pat = req(sensor, "pattern", "sensor");
  const auto type = reqAs<std::string>(pat, "type", "sensor.pattern");
  if (type == "sparse") c.sensor.pattern.type = PdPatternDesc::Type::kSparse;
  else if (type == "dual_pd") c.sensor.pattern.type = PdPatternDesc::Type::kDualPd;
  else throw std::runtime_error("config: sensor.pattern.type must be 'sparse' or 'dual_pd'");
  c.sensor.pattern.pair_pitch_x = reqAs<int>(pat, "pair_pitch_x", "sensor.pattern");
  c.sensor.pattern.pair_pitch_y = reqAs<int>(pat, "pair_pitch_y", "sensor.pattern");
  const auto& roi = req(sensor, "default_roi", "sensor");
  c.sensor.default_roi = {reqAs<int>(roi, "x", "sensor.default_roi"),
                          reqAs<int>(roi, "y", "sensor.default_roi"),
                          reqAs<int>(roi, "w", "sensor.default_roi"),
                          reqAs<int>(roi, "h", "sensor.default_roi")};
  c.sensor.roi_sample_width = reqAs<int>(sensor, "roi_sample_width", "sensor");
  c.sensor.roi_sample_height = reqAs<int>(sensor, "roi_sample_height", "sensor");

  const auto& calib = req(j, "calibration", "");
  const auto& lrc = req(calib, "lrc", "calibration");
  c.calibration.lrc.left_gain = reqAs<float>(lrc, "left_gain", "calibration.lrc");
  c.calibration.lrc.right_gain = reqAs<float>(lrc, "right_gain", "calibration.lrc");
  const auto& dcc = req(calib, "dcc", "calibration");
  c.calibration.dcc.step_min = reqAs<int>(dcc, "step_min", "calibration.dcc");
  c.calibration.dcc.step_max = reqAs<int>(dcc, "step_max", "calibration.dcc");
  for (const auto& a : req(dcc, "anchors", "calibration.dcc")) {
    c.calibration.dcc.anchors.push_back(
        {reqAs<int>(a, "step", "calibration.dcc.anchors[]"),
         reqAs<float>(a, "steps_per_disparity", "calibration.dcc.anchors[]")});
  }
  if (c.calibration.dcc.anchors.empty())
    throw std::runtime_error("config: calibration.dcc.anchors must not be empty");

  const auto& tun = req(j, "tuning", "");
  c.tuning.shift_min = reqAs<int>(tun, "shift_min", "tuning");
  c.tuning.shift_max = reqAs<int>(tun, "shift_max", "tuning");
  c.tuning.confidence_threshold = reqAs<float>(tun, "confidence_threshold", "tuning");
  c.tuning.in_focus_disparity = reqAs<float>(tun, "in_focus_disparity", "tuning");
  c.tuning.max_retries = reqAs<int>(tun, "max_retries", "tuning");
  c.tuning.max_iterations = reqAs<int>(tun, "max_iterations", "tuning");

  const auto& sys = req(j, "system", "");
  c.system.mode = reqAs<std::string>(sys, "mode", "system");
  if (c.system.mode != "sim" && c.system.mode != "replay")
    throw std::runtime_error("config: system.mode must be 'sim' or 'replay'");
  c.system.log_dir = reqAs<std::string>(sys, "log_dir", "system");
  c.system.replay_dir = reqAs<std::string>(sys, "replay_dir", "system");
  const auto& sim = req(sys, "sim", "system");
  c.system.sim.object_distance_mm = reqAs<double>(sim, "object_distance_mm", "system.sim");
  c.system.sim.initial_step = reqAs<int>(sim, "initial_step", "system.sim");
  c.system.sim.noise_sigma = reqAs<float>(sim, "noise_sigma", "system.sim");
  c.system.sim.gain_mismatch = reqAs<float>(sim, "gain_mismatch", "system.sim");
  c.system.sim.settle_frames = reqAs<int>(sim, "settle_frames", "system.sim");
  c.system.sim.seed = reqAs<uint32_t>(sim, "seed", "system.sim");
  c.system.sim.step_inf = reqAs<int>(sim, "step_inf", "system.sim");
  c.system.sim.focus_gain = reqAs<float>(sim, "focus_gain", "system.sim");

  return c;
}

AfConfig AfConfig::loadFromFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("config: cannot open file: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return loadFromJson(ss.str());
}

}  // namespace pdaf
```

- [ ] **Step 5: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 AfConfig 4 條）

- [ ] **Step 6: Commit**

```bash
git add include/pdaf/control/af_config.h src/control/ config/ tests/ CMakeLists.txt
git commit -m "feat: AfConfig JSON 載入（fail-fast 指出欄位）與 default.json"
```

---

### Task 7: PdafPipeline（IFocusEstimator + raw/HW 雙路徑）

**Files:**
- Create: `include/pdaf/control/focus_estimator.h`, `src/control/pdaf_pipeline.h`, `src/control/pdaf_pipeline.cpp`
- Test: `tests/test_pdaf_pipeline.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/control/pdaf_pipeline.cpp`）、`tests/CMakeLists.txt`（加 `test_pdaf_pipeline.cpp`）

**Interfaces:**
- Consumes: `IPdCostEngine`（Task 3）、`IDepthEstimator`（Task 2）、`PdInput`（Task 1）
- Produces: `class IFocusEstimator { virtual std::vector<DepthEstimate> process(const PdInput&) = 0; }`；`class PdafPipeline : public IFocusEstimator { PdafPipeline(std::unique_ptr<IPdCostEngine>, std::unique_ptr<IDepthEstimator>); }`（HAF 式仲裁縫：controller 只認 IFocusEstimator）

- [ ] **Step 1: 寫失敗測試 `tests/test_pdaf_pipeline.cpp`**

```cpp
#include <gtest/gtest.h>
#include <control/pdaf_pipeline.h>

#include <algorithm>

using namespace pdaf;

namespace {

// stub M1：回傳固定 cost；記錄是否被呼叫
class StubCostEngine : public IPdCostEngine {
 public:
  bool called = false;
  void init(const LrcCalib&, const PdPatternDesc&, int, int) override {}
  std::vector<CostSequence> compute(const PdFrame& f) override {
    called = true;
    return std::vector<CostSequence>(f.rois.size(), CostSequence{-2, {3, 1, 3, 5, 7}, 10});
  }
};

// stub M2：disparity = shift 極小值位置
class StubEstimator : public IDepthEstimator {
 public:
  DepthEstimate estimate(const CostSequence& c) override {
    auto mi = std::min_element(c.costs.begin(), c.costs.end()) - c.costs.begin();
    return {static_cast<float>(c.shift_min + mi), 0.9f, true};
  }
};

}  // namespace

TEST(PdafPipeline, RawPathRunsM1ThenM2) {
  auto m1 = std::make_unique<StubCostEngine>();
  auto* m1p = m1.get();
  PdafPipeline pipe(std::move(m1), std::make_unique<StubEstimator>());
  PdInput in;
  PdFrame f;
  f.rois.resize(2);
  in.raw = f;
  auto out = pipe.process(in);
  EXPECT_TRUE(m1p->called);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_FLOAT_EQ(out[0].disparity, -1.f);  // min at index 1 → -2+1
}

TEST(PdafPipeline, HwCostPathBypassesM1) {
  auto m1 = std::make_unique<StubCostEngine>();
  auto* m1p = m1.get();
  PdafPipeline pipe(std::move(m1), std::make_unique<StubEstimator>());
  PdInput in;
  in.hw_costs = std::vector<CostSequence>{CostSequence{0, {5, 1, 5}, 10}};
  auto out = pipe.process(in);
  EXPECT_FALSE(m1p->called);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].disparity, 1.f);
}

TEST(PdafPipeline, EmptyInputGivesNoEstimates) {
  PdafPipeline pipe(std::make_unique<StubCostEngine>(), std::make_unique<StubEstimator>());
  EXPECT_TRUE(pipe.process(PdInput{}).empty());  // 不丟例外
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `control/pdaf_pipeline.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/control/focus_estimator.h`：

```cpp
#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf {
// 對焦估測技術的抽象（HAF 式仲裁縫）：PDAF 是其中一種實作，
// 將來 contrast/TOF 也以此介面掛入，controller 不需修改。
class IFocusEstimator {
 public:
  virtual ~IFocusEstimator() = default;
  virtual std::vector<DepthEstimate> process(const PdInput& input) = 0;
};
}  // namespace pdaf
```

`src/control/pdaf_pipeline.h`：

```cpp
#pragma once
#include <pdaf/algo/depth_estimator.h>
#include <pdaf/algo/pd_cost_engine.h>
#include <pdaf/control/focus_estimator.h>

#include <memory>

namespace pdaf {
// M1+M2 組合；hw_costs 存在時走 HW 統計路徑（M1 被 ISP 硬體取代的情境）
class PdafPipeline : public IFocusEstimator {
 public:
  PdafPipeline(std::unique_ptr<IPdCostEngine> cost_engine,
               std::unique_ptr<IDepthEstimator> depth_estimator);
  std::vector<DepthEstimate> process(const PdInput& input) override;

 private:
  std::unique_ptr<IPdCostEngine> cost_engine_;
  std::unique_ptr<IDepthEstimator> depth_estimator_;
};
}  // namespace pdaf
```

`src/control/pdaf_pipeline.cpp`：

```cpp
#include "control/pdaf_pipeline.h"

namespace pdaf {

PdafPipeline::PdafPipeline(std::unique_ptr<IPdCostEngine> cost_engine,
                           std::unique_ptr<IDepthEstimator> depth_estimator)
    : cost_engine_(std::move(cost_engine)),
      depth_estimator_(std::move(depth_estimator)) {}

std::vector<DepthEstimate> PdafPipeline::process(const PdInput& input) {
  std::vector<CostSequence> costs;
  if (input.hw_costs) costs = *input.hw_costs;
  else if (input.raw) costs = cost_engine_->compute(*input.raw);

  std::vector<DepthEstimate> out;
  out.reserve(costs.size());
  for (const auto& c : costs) out.push_back(depth_estimator_->estimate(c));
  return out;
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 PdafPipeline 3 條）

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/control/focus_estimator.h src/control/pdaf_pipeline.* tests/ CMakeLists.txt
git commit -m "feat: PdafPipeline（IFocusEstimator、raw/HW 雙路徑）"
```

---

### Task 8: AfController 狀態機

**Files:**
- Create: `include/pdaf/control/af_controller.h`, `src/control/af_controller.cpp`
- Test: `tests/test_af_controller.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/control/af_controller.cpp`）、`tests/CMakeLists.txt`（加 `test_af_controller.cpp`）

**Interfaces:**
- Consumes: `IFocusEstimator`（Task 7）、`ILensMapper`（Task 4）、`ILensActuator`（Task 5）、`TuningConfig`（Task 6）
- Produces:

```cpp
enum class AfState { kIdle, kMeasuring, kMoving, kSettling, kVerifying, kFocused, kFailed };
const char* toString(AfState);
struct AfFrameLog { uint64_t frame_id; AfState state_before; AfState state_after;
                    float disparity; float confidence; int lens_step; int target_step; };
class AfController {
 public:
  AfController(IFocusEstimator&, ILensMapper&, ILensActuator&, const TuningConfig&);
  void trigger();                // Idle/Focused/Failed → Measuring；重置計數
  AfState state() const;
  AfFrameLog onFrame(const AfRequest&, const PdInput&);
};
```

狀態機規則（spec 的落實）：
- `kMeasuring`/`kVerifying`：跑 estimator（取第一個 ROI 為 primary）。`!valid || conf < threshold` → retry++，超過 `max_retries` → `kFailed`。有效且 `|disparity| < in_focus_disparity` → `kFocused`。否則 iteration++（超過 `max_iterations` → `kFailed`），用 `meta.lens_step_at_exposure` 呼叫 M3、下 `moveTo`、進 `kMoving`
- `kMoving`：下一個 frame 轉 `kSettling`（順帶檢查 moving）
- `kSettling`：`!getStatus().moving` → `kVerifying`
- `kIdle`/`kFocused`/`kFailed`：onFrame 不動作，只記 log

- [ ] **Step 1: 寫失敗測試 `tests/test_af_controller.cpp`**

```cpp
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
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `pdaf/control/af_controller.h: No such file or directory`

- [ ] **Step 3: 實作**

`include/pdaf/control/af_controller.h`：

```cpp
#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/control/af_config.h>
#include <pdaf/control/focus_estimator.h>
#include <pdaf/hal/lens_actuator.h>

namespace pdaf {

enum class AfState { kIdle, kMeasuring, kMoving, kSettling, kVerifying, kFocused, kFailed };
const char* toString(AfState s);

struct AfFrameLog {
  uint64_t frame_id = 0;
  AfState state_before = AfState::kIdle;
  AfState state_after = AfState::kIdle;
  float disparity = 0.f;
  float confidence = 0.f;
  int lens_step = 0;
  int target_step = 0;
};

// 單次 AF 狀態機。逐 frame 由 onFrame() 驅動（產品 AF 的實際型態）。
// CAF 擴充點：kFocused 後加場景監測轉移即可，不需改架構。
class AfController {
 public:
  AfController(IFocusEstimator& estimator, ILensMapper& mapper,
               ILensActuator& actuator, const TuningConfig& tuning);
  void trigger();
  AfState state() const { return state_; }
  AfFrameLog onFrame(const AfRequest& request, const PdInput& input);

 private:
  void handleMeasurement(const PdInput& input, AfFrameLog& log);

  IFocusEstimator& estimator_;
  ILensMapper& mapper_;
  ILensActuator& actuator_;
  TuningConfig tuning_;
  AfState state_ = AfState::kIdle;
  int retries_ = 0;
  int iterations_ = 0;
  int target_step_ = 0;
};

}  // namespace pdaf
```

`src/control/af_controller.cpp`：

```cpp
#include <pdaf/control/af_controller.h>

#include <cmath>

namespace pdaf {

const char* toString(AfState s) {
  switch (s) {
    case AfState::kIdle: return "IDLE";
    case AfState::kMeasuring: return "MEASURING";
    case AfState::kMoving: return "MOVING";
    case AfState::kSettling: return "SETTLING";
    case AfState::kVerifying: return "VERIFYING";
    case AfState::kFocused: return "FOCUSED";
    case AfState::kFailed: return "FAILED";
  }
  return "?";
}

AfController::AfController(IFocusEstimator& estimator, ILensMapper& mapper,
                           ILensActuator& actuator, const TuningConfig& tuning)
    : estimator_(estimator), mapper_(mapper), actuator_(actuator), tuning_(tuning) {}

void AfController::trigger() {
  if (state_ == AfState::kIdle || state_ == AfState::kFocused ||
      state_ == AfState::kFailed) {
    retries_ = 0;
    iterations_ = 0;
    state_ = AfState::kMeasuring;
  }
}

AfFrameLog AfController::onFrame(const AfRequest&, const PdInput& input) {
  AfFrameLog log;
  log.frame_id = input.meta.frame_id;
  log.state_before = state_;
  log.lens_step = actuator_.getStatus().current_step;
  log.target_step = target_step_;

  switch (state_) {
    case AfState::kIdle:
    case AfState::kFocused:
    case AfState::kFailed:
      break;
    case AfState::kMoving:
      state_ = AfState::kSettling;
      [[fallthrough]];
    case AfState::kSettling:
      if (!actuator_.getStatus().moving) state_ = AfState::kVerifying;
      break;
    case AfState::kMeasuring:
    case AfState::kVerifying:
      handleMeasurement(input, log);
      break;
  }

  log.state_after = state_;
  log.target_step = target_step_;
  return log;
}

void AfController::handleMeasurement(const PdInput& input, AfFrameLog& log) {
  const auto ests = estimator_.process(input);
  const DepthEstimate e = ests.empty() ? DepthEstimate{} : ests.front();  // primary ROI
  log.disparity = e.disparity;
  log.confidence = e.confidence;

  if (!e.valid || e.confidence < tuning_.confidence_threshold) {
    if (++retries_ > tuning_.max_retries) state_ = AfState::kFailed;
    return;  // 低信心：留在原狀態重量測（執行期不丟例外，走降級路徑）
  }
  retries_ = 0;

  if (std::abs(e.disparity) < tuning_.in_focus_disparity) {
    state_ = AfState::kFocused;
    return;
  }
  if (++iterations_ > tuning_.max_iterations) {
    state_ = AfState::kFailed;
    return;
  }
  // 基準必須用曝光當下的 lens 位置（實機 stats 有 pipeline 延遲）
  const LensCommand cmd = mapper_.toLensCommand(e, input.meta.lens_step_at_exposure);
  target_step_ = cmd.target_step;
  actuator_.moveTo(cmd.target_step);
  state_ = AfState::kMoving;
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 AfController 5 條）

- [ ] **Step 5: Commit**

```bash
git add include/pdaf/control/af_controller.h src/control/af_controller.cpp tests/ CMakeLists.txt
git commit -m "feat: AfController 單次 AF 狀態機（retry/iteration 上限、曝光位置基準）"
```

---

### Task 9: 光學模擬器（SimWorld + HAL adapters）

**Files:**
- Create: `src/sim/sim_world.h`, `src/sim/sim_world.cpp`
- Test: `tests/test_sim_world.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/sim/sim_world.cpp`）、`tests/CMakeLists.txt`（加 `test_sim_world.cpp`）

**Interfaces:**
- Consumes: `SimConfig`, `SensorConfig`（Task 6）、`DccTable`/`dccInterp`（Task 4）、HAL 介面（Task 5）
- Produces:

```cpp
class SimWorld {
 public:
  SimWorld(const SensorConfig&, const SimConfig&, const DccTable&);
  PdInput capture(const AfRequest&);  // 前進一個 frame（含 actuator 運動）並產生資料
  void moveTo(int step);
  LensStatus lensStatus() const;
  int inFocusStep() const;   // step_inf + focus_gain / object_distance_mm（clamp 到 step 範圍）
  int currentStep() const;
  float groundTruthDisparity() const;  // (inFocusStep - currentStep) / dccInterp(current)
};
class SimPdDataSource : public IPdDataSource;   // 轉呼叫 SimWorld::capture
class SimLensActuator : public ILensActuator;   // 轉呼叫 SimWorld::moveTo / lensStatus
```

物理模型：
- 合焦位置：`inFocusStep = clamp(step_inf + focus_gain / object_distance_mm, step_min, step_max)`
- 真值 disparity 由 DCC 反推（`dccInterp` 與 M3 共用 → 閉環自洽）
- 訊號：連續紋理函式（seed 決定相位）`tex(x) = 2 + 0.8·sin(0.37x+p1) + 0.5·sin(1.13x+p2) + 0.3·sin(2.71x+p3)`；`L[i] = blur(tex)(i + y·7)`、`R[i] = blur(tex)(i + y·7 − d_gt) · gain_mismatch + noise`；blur 為半徑 `min(6, |defocus_steps|/25)` 的移動平均（離焦越多對比越低）
- actuator：`moveTo` 設定 target，之後 `settle_frames` 次 `capture` 期間 `moving=true`，之後跳到 target
- `capture` 每次 `frame_id++`，`lens_step_at_exposure = currentStep()`（運動中取移動前位置——demo 簡化，運動中的量測會被 SETTLING 擋掉）

- [ ] **Step 1: 寫失敗測試 `tests/test_sim_world.cpp`**

```cpp
#include <gtest/gtest.h>
#include <sim/sim_world.h>

#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>

#include <cmath>

using namespace pdaf;

namespace {

SensorConfig sensorCfg() {
  SensorConfig s;
  s.roi_sample_width = 64;
  s.roi_sample_height = 4;
  return s;
}

SimConfig simCfg(double dist_mm, int init_step) {
  SimConfig c;
  c.object_distance_mm = dist_mm;
  c.initial_step = init_step;
  c.noise_sigma = 0.005f;
  c.gain_mismatch = 1.1f;
  c.settle_frames = 3;
  c.seed = 42;
  return c;  // step_inf=100, focus_gain=150000（預設）
}

DccTable dcc() { return DccTable{0, 1023, {{0, 50.f}, {1023, 50.f}}}; }

}  // namespace

TEST(SimWorld, InFocusStepFromDistance) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  EXPECT_EQ(w.inFocusStep(), 175);  // 100 + 150000/2000
  EXPECT_EQ(w.currentStep(), 300);
  EXPECT_NEAR(w.groundTruthDisparity(), (175 - 300) / 50.f, 0.01f);
}

TEST(SimWorld, ActuatorSettlesAfterConfiguredFrames) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  w.moveTo(500);
  AfRequest req{{Roi{0, 0, 64, 4}}};
  EXPECT_TRUE(w.lensStatus().moving);
  w.capture(req);
  w.capture(req);
  EXPECT_TRUE(w.lensStatus().moving);
  w.capture(req);  // 第 settle_frames 次 → 到位
  EXPECT_FALSE(w.lensStatus().moving);
  EXPECT_EQ(w.currentStep(), 500);
}

TEST(SimWorld, CaptureMetaCarriesFrameIdAndExposureStep) {
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());
  AfRequest req{{Roi{0, 0, 64, 4}}};
  auto a = w.capture(req);
  auto b = w.capture(req);
  EXPECT_EQ(b.meta.frame_id, a.meta.frame_id + 1);
  EXPECT_EQ(a.meta.lens_step_at_exposure, 300);
  ASSERT_TRUE(a.raw.has_value());
  ASSERT_EQ(a.raw->rois.size(), 1u);
  EXPECT_EQ(a.raw->rois[0].width, 64);
}

TEST(SimWorld, PipelineMeasuresGroundTruthDisparity) {
  // 模擬器資料丟進 M1+M2，量到的 disparity 應接近真值
  SimWorld w(sensorCfg(), simCfg(2000.0, 300), dcc());  // d_gt = -2.5
  AfRequest req{{Roi{0, 0, 64, 4}}};
  auto in = w.capture(req);

  SadCostEngine m1;
  m1.init(LrcCalib{1.f, 1.f / 1.1f}, PdPatternDesc{}, -16, 16);  // 校正抵銷 gain_mismatch
  auto costs = m1.compute(*in.raw);
  auto e = ParabolicDepthEstimator{}.estimate(costs[0]);
  EXPECT_TRUE(e.valid);
  EXPECT_GT(e.confidence, 0.3f);
  EXPECT_NEAR(e.disparity, w.groundTruthDisparity(), 0.5f);
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `sim/sim_world.h: No such file or directory`

- [ ] **Step 3: 實作**

`src/sim/sim_world.h`：

```cpp
#pragma once
#include <pdaf/algo/lens_mapper.h>
#include <pdaf/control/af_config.h>
#include <pdaf/hal/lens_actuator.h>
#include <pdaf/hal/pd_data_source.h>
#include <pdaf/types.h>

#include <random>

namespace pdaf {

// 薄透鏡閉環模擬：VCM step ↔ defocus ↔ L/R 相位偏移
class SimWorld {
 public:
  SimWorld(const SensorConfig& sensor, const SimConfig& sim, const DccTable& dcc);
  PdInput capture(const AfRequest& request);  // 每次呼叫前進一個 frame
  void moveTo(int step);
  LensStatus lensStatus() const;
  int inFocusStep() const;
  int currentStep() const { return current_step_; }
  float groundTruthDisparity() const;

 private:
  float texture(float x) const;      // 連續紋理（seed 決定相位）
  float blurredTexture(float x, int radius) const;

  SensorConfig sensor_;
  SimConfig sim_;
  DccTable dcc_;
  int current_step_;
  int target_step_;
  int settle_counter_ = 0;
  uint64_t frame_id_ = 0;
  float phase_[3];
  mutable std::mt19937 rng_;
};

class SimPdDataSource : public IPdDataSource {
 public:
  explicit SimPdDataSource(SimWorld& world) : world_(world) {}
  PdInput capture(const AfRequest& request) override { return world_.capture(request); }

 private:
  SimWorld& world_;
};

class SimLensActuator : public ILensActuator {
 public:
  explicit SimLensActuator(SimWorld& world) : world_(world) {}
  void moveTo(int step) override { world_.moveTo(step); }
  LensStatus getStatus() const override { return world_.lensStatus(); }

 private:
  SimWorld& world_;
};

}  // namespace pdaf
```

`src/sim/sim_world.cpp`：

```cpp
#include "sim/sim_world.h"

#include <algorithm>
#include <cmath>

namespace pdaf {

SimWorld::SimWorld(const SensorConfig& sensor, const SimConfig& sim, const DccTable& dcc)
    : sensor_(sensor), sim_(sim), dcc_(dcc),
      current_step_(sim.initial_step), target_step_(sim.initial_step),
      rng_(sim.seed) {
  std::uniform_real_distribution<float> u(0.f, 6.28f);
  for (auto& p : phase_) p = u(rng_);
}

int SimWorld::inFocusStep() const {
  const int s = sim_.step_inf +
      static_cast<int>(std::lround(sim_.focus_gain / sim_.object_distance_mm));
  return std::clamp(s, dcc_.step_min, dcc_.step_max);
}

float SimWorld::groundTruthDisparity() const {
  return (inFocusStep() - current_step_) / dccInterp(dcc_, current_step_);
}

void SimWorld::moveTo(int step) {
  target_step_ = std::clamp(step, dcc_.step_min, dcc_.step_max);
  if (target_step_ != current_step_) settle_counter_ = sim_.settle_frames;
}

LensStatus SimWorld::lensStatus() const { return {current_step_, settle_counter_ > 0}; }

float SimWorld::texture(float x) const {
  // 頻率皆低於 π/16 (~0.196 rad/sample)，確保 SAD cost 在 ±16 shift 搜尋窗內單峰、
  // 不與搜尋窗產生 aliasing（高頻紋理的週期若 ≈ 搜尋窗寬會出現假極小值）
  return 2.f + 0.7f * std::sin(0.07f * x + phase_[0]) +
         0.6f * std::sin(0.13f * x + phase_[1]) +
         0.4f * std::sin(0.19f * x + phase_[2]);
}

float SimWorld::blurredTexture(float x, int radius) const {
  if (radius <= 0) return texture(x);
  float acc = 0.f;
  for (int j = -radius; j <= radius; ++j) acc += texture(x + static_cast<float>(j));
  return acc / static_cast<float>(2 * radius + 1);
}

PdInput SimWorld::capture(const AfRequest& request) {
  // frame 前進：actuator 運動
  if (settle_counter_ > 0 && --settle_counter_ == 0) current_step_ = target_step_;

  const float d = groundTruthDisparity();
  const int defocus = std::abs(inFocusStep() - current_step_);
  const int radius = std::min(6, defocus / 25);
  std::normal_distribution<float> noise(0.f, sim_.noise_sigma);

  PdInput in;
  in.meta = {frame_id_++, static_cast<double>(frame_id_) * 33.3, current_step_};

  PdFrame f;
  f.meta = in.meta;
  f.pattern = sensor_.pattern;
  const int n = sensor_.roi_sample_width;
  const int h = sensor_.roi_sample_height;
  const size_t roi_count = std::max<size_t>(1, request.rois.size());
  for (size_t r = 0; r < roi_count; ++r) {
    RoiSamples s;
    s.width = n;
    s.height = h;
    const float roi_offset = static_cast<float>(r) * 131.f;  // 各 ROI 看不同紋理區
    for (int y = 0; y < h; ++y) {
      for (int i = 0; i < n; ++i) {
        const float x = static_cast<float>(i) + static_cast<float>(y) * 7.f + roi_offset;
        s.left.push_back(blurredTexture(x, radius));
        s.right.push_back(blurredTexture(x - d, radius) * sim_.gain_mismatch + noise(rng_));
      }
    }
    f.rois.push_back(std::move(s));
  }
  in.raw = std::move(f);
  return in;
}

}  // namespace pdaf
```

（M1 的 cost 定義是 `|L[i] − R[i+s]|`，`R[i] = tex(i − d)` → `R[i+s] = tex(i+s−d)`，`s = d` 時對齊——極小值落在 `+d`，與 `groundTruthDisparity` 同號。）

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 SimWorld 4 條）

- [ ] **Step 5: Commit**

```bash
git add src/sim/ tests/ CMakeLists.txt
git commit -m "feat: 光學模擬器 SimWorld（薄透鏡閉環、settle、noise/gain/blur）"
```

---

### Task 10: Replay 資料來源（JSON frame dump）

**Files:**
- Create: `src/replay/replay_pd_data_source.h`, `src/replay/replay_pd_data_source.cpp`
- Test: `tests/test_replay_source.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/replay/replay_pd_data_source.cpp`）、`tests/CMakeLists.txt`（加 `test_replay_source.cpp`）

**Interfaces:**
- Consumes: `IPdDataSource`（Task 5）、`PdInput`（Task 1）
- Produces: `class ReplayPdDataSource : public IPdDataSource { explicit ReplayPdDataSource(const std::string& dir); size_t frameCount() const; }`——載入 `dir` 下 `frame_0000.json`、`frame_0001.json`…（連號，遇缺號停止）；`capture()` 依序回放，放完重複最後一筆

Frame 檔格式（raw 與 hw_costs 擇一，對應 `PdInput` variant）：

```json
{
  "meta": { "frame_id": 0, "timestamp_ms": 0.0, "lens_step_at_exposure": 300 },
  "rois": [ { "width": 4, "height": 1, "left": [1,2,3,4], "right": [1,2,3,4] } ]
}
```

或 HW 統計路徑：

```json
{
  "meta": { "frame_id": 0, "timestamp_ms": 0.0, "lens_step_at_exposure": 300 },
  "hw_costs": [ { "shift_min": -2, "costs": [5,1,5,7,9], "valid_samples": 10 } ]
}
```

- [ ] **Step 1: 寫失敗測試 `tests/test_replay_source.cpp`**

```cpp
#include <gtest/gtest.h>
#include <replay/replay_pd_data_source.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace pdaf;
namespace fs = std::filesystem;

namespace {

class ReplayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / "pdaf_replay_test";
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }
  void writeFrame(int idx, const std::string& body) {
    char name[32];
    std::snprintf(name, sizeof(name), "frame_%04d.json", idx);
    std::ofstream(dir_ / name) << body;
  }
  fs::path dir_;
};

}  // namespace

TEST_F(ReplayTest, LoadsRawFramesInOrder) {
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
  auto c = src.capture(AfRequest{});  // 放完重複最後一筆
  EXPECT_EQ(c.meta.frame_id, 1u);
}

TEST_F(ReplayTest, LoadsHwCostFrames) {
  writeFrame(0, R"({"meta":{"frame_id":0,"timestamp_ms":0.0,"lens_step_at_exposure":300},
    "hw_costs":[{"shift_min":-2,"costs":[5,1,5,7,9],"valid_samples":10}]})");
  ReplayPdDataSource src(dir_.string());
  auto a = src.capture(AfRequest{});
  EXPECT_FALSE(a.raw.has_value());
  ASSERT_TRUE(a.hw_costs.has_value());
  EXPECT_EQ((*a.hw_costs)[0].shift_min, -2);
}

TEST_F(ReplayTest, EmptyDirThrows) {
  EXPECT_THROW(ReplayPdDataSource(dir_.string()), std::runtime_error);
}
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `replay/replay_pd_data_source.h: No such file or directory`

- [ ] **Step 3: 實作**

`src/replay/replay_pd_data_source.h`：

```cpp
#pragma once
#include <pdaf/hal/pd_data_source.h>

#include <string>
#include <vector>

namespace pdaf {
// 讀取 dir 下 frame_0000.json 起的連號 dump，依序回放；放完重複最後一筆。
// 用途：單模塊驗證、真機資料離線分析。
class ReplayPdDataSource : public IPdDataSource {
 public:
  explicit ReplayPdDataSource(const std::string& dir);  // 無任何 frame → throw
  PdInput capture(const AfRequest& request) override;
  size_t frameCount() const { return frames_.size(); }

 private:
  std::vector<PdInput> frames_;
  size_t next_ = 0;
};
}  // namespace pdaf
```

`src/replay/replay_pd_data_source.cpp`：

```cpp
#include "replay/replay_pd_data_source.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace pdaf {
namespace {

using nlohmann::json;

PdInput parseFrame(const json& j, const std::string& file) {
  PdInput in;
  const auto& m = j.at("meta");
  in.meta.frame_id = m.at("frame_id").get<uint64_t>();
  in.meta.timestamp_ms = m.at("timestamp_ms").get<double>();
  in.meta.lens_step_at_exposure = m.at("lens_step_at_exposure").get<int>();

  if (j.contains("hw_costs")) {
    std::vector<CostSequence> costs;
    for (const auto& c : j["hw_costs"]) {
      costs.push_back({c.at("shift_min").get<int>(),
                       c.at("costs").get<std::vector<float>>(),
                       c.at("valid_samples").get<int>()});
    }
    in.hw_costs = std::move(costs);
  } else if (j.contains("rois")) {
    PdFrame f;
    f.meta = in.meta;
    for (const auto& r : j["rois"]) {
      RoiSamples s;
      s.width = r.at("width").get<int>();
      s.height = r.at("height").get<int>();
      s.left = r.at("left").get<std::vector<float>>();
      s.right = r.at("right").get<std::vector<float>>();
      f.rois.push_back(std::move(s));
    }
    in.raw = std::move(f);
  } else {
    throw std::runtime_error("replay: frame has neither 'rois' nor 'hw_costs': " + file);
  }
  return in;
}

}  // namespace

ReplayPdDataSource::ReplayPdDataSource(const std::string& dir) {
  for (int idx = 0;; ++idx) {
    char name[32];
    std::snprintf(name, sizeof(name), "frame_%04d.json", idx);
    std::ifstream f(dir + "/" + name);
    if (!f) break;
    json j;
    try {
      f >> j;
      frames_.push_back(parseFrame(j, name));
    } catch (const json::exception& e) {
      throw std::runtime_error(std::string("replay: bad frame ") + name + ": " + e.what());
    }
  }
  if (frames_.empty())
    throw std::runtime_error("replay: no frame_0000.json found in " + dir);
}

PdInput ReplayPdDataSource::capture(const AfRequest&) {
  const size_t i = std::min(next_, frames_.size() - 1);
  if (next_ + 1 < frames_.size()) ++next_;
  return frames_[i];
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 Replay 3 條）

- [ ] **Step 5: Commit**

```bash
git add src/replay/ tests/ CMakeLists.txt
git commit -m "feat: ReplayPdDataSource（JSON frame dump 重播、raw/hw 雙格式）"
```

---

### Task 11: 執行紀錄（frames.csv + summary.txt）

**Files:**
- Create: `src/control/run_logger.h`, `src/control/run_logger.cpp`
- Test: `tests/test_run_logger.cpp`
- Modify: 根 `CMakeLists.txt`（加 `src/control/run_logger.cpp`）、`tests/CMakeLists.txt`（加 `test_run_logger.cpp`）

**Interfaces:**
- Consumes: `AfFrameLog`, `toString(AfState)`（Task 8）
- Produces:

```cpp
class RunLogger {
 public:
  explicit RunLogger(const std::string& out_dir);  // 建目錄、開 frames.csv 寫 header
  void logFrame(const AfFrameLog& log, float gt_disparity = 0.f);  // sim 模式帶真值，replay 帶 0
  void writeSummary(AfState final_state, int total_frames, int final_step,
                    int in_focus_step);  // summary.txt；in_focus_step<0 表示無真值
};
```

`frames.csv` 欄位：`frame_id,state_before,state_after,disparity,confidence,lens_step,target_step,gt_disparity`

- [ ] **Step 1: 寫失敗測試 `tests/test_run_logger.cpp`**

```cpp
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
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build build`
Expected: FAIL — `control/run_logger.h: No such file or directory`

- [ ] **Step 3: 實作**

`src/control/run_logger.h`：

```cpp
#pragma once
#include <pdaf/control/af_controller.h>

#include <fstream>
#include <string>

namespace pdaf {
// 逐 frame CSV + 收斂摘要（可直接用 Python/gnuplot 繪圖分析）
class RunLogger {
 public:
  explicit RunLogger(const std::string& out_dir);
  void logFrame(const AfFrameLog& log, float gt_disparity = 0.f);
  void writeSummary(AfState final_state, int total_frames, int final_step,
                    int in_focus_step);

 private:
  std::string dir_;
  std::ofstream csv_;
};
}  // namespace pdaf
```

`src/control/run_logger.cpp`：

```cpp
#include "control/run_logger.h"

#include <cmath>
#include <filesystem>

namespace pdaf {

RunLogger::RunLogger(const std::string& out_dir) : dir_(out_dir) {
  std::filesystem::create_directories(dir_);
  csv_.open(dir_ + "/frames.csv");
  csv_ << "frame_id,state_before,state_after,disparity,confidence,"
          "lens_step,target_step,gt_disparity\n";
}

void RunLogger::logFrame(const AfFrameLog& log, float gt_disparity) {
  csv_ << log.frame_id << ',' << toString(log.state_before) << ','
       << toString(log.state_after) << ',' << log.disparity << ','
       << log.confidence << ',' << log.lens_step << ',' << log.target_step << ','
       << gt_disparity << '\n';
}

void RunLogger::writeSummary(AfState final_state, int total_frames, int final_step,
                             int in_focus_step) {
  std::ofstream f(dir_ + "/summary.txt");
  f << "final_state: " << toString(final_state) << '\n'
    << "total_frames: " << total_frames << '\n'
    << "final_step: " << final_step << '\n';
  if (in_focus_step >= 0) {
    f << "in_focus_step: " << in_focus_step << '\n'
      << "final_error_steps: " << std::abs(final_step - in_focus_step) << '\n';
  }
}

}  // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS

- [ ] **Step 5: Commit**

```bash
git add src/control/run_logger.* tests/ CMakeLists.txt
git commit -m "feat: RunLogger（frames.csv 逐 frame 紀錄 + summary.txt）"
```

---

### Task 12: 閉環整合測試 + CLI

**Files:**
- Create: `tests/test_closed_loop.cpp`, `apps/pdaf_cli/main.cpp`, `apps/CMakeLists.txt`
- Modify: 根 `CMakeLists.txt`（加 `add_subdirectory(apps)`）、`tests/CMakeLists.txt`（加 `test_closed_loop.cpp`）

**Interfaces:**
- Consumes: 全部前述元件
- Produces: `pdaf_cli` 執行檔（`--config <path> [--mode sim|replay] [--out <dir>] [--max-frames N]`）

- [ ] **Step 1: 寫失敗（先寫閉環整合測試）`tests/test_closed_loop.cpp`**

```cpp
#include <gtest/gtest.h>

#include <algo/dcc_lens_mapper.h>
#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>
#include <control/pdaf_pipeline.h>
#include <pdaf/control/af_controller.h>
#include <sim/sim_world.h>

#include <cmath>

using namespace pdaf;

namespace {

struct RunResult {
  AfState final_state;
  int frames_used;
  int final_error_steps;
};

// 框架的「活規格」：模擬器 + 全 pipeline 閉環
RunResult runScenario(double dist_mm, int init_step, float noise_sigma) {
  SensorConfig sensor;
  sensor.roi_sample_width = 64;
  sensor.roi_sample_height = 4;

  SimConfig sim;
  sim.object_distance_mm = dist_mm;
  sim.initial_step = init_step;
  sim.noise_sigma = noise_sigma;
  sim.gain_mismatch = 1.1f;
  sim.settle_frames = 3;
  sim.seed = 42;

  DccTable dcc{0, 1023, {{0, 50.f}, {1023, 50.f}}};
  TuningConfig tun{-16, 16, 0.3f, 0.25f, 3, 6};

  SimWorld world(sensor, sim, dcc);
  SimPdDataSource source(world);
  SimLensActuator actuator(world);

  auto m1 = std::make_unique<SadCostEngine>();
  m1->init(LrcCalib{1.f, 1.f / 1.1f}, sensor.pattern, tun.shift_min, tun.shift_max);
  PdafPipeline pipeline(std::move(m1), std::make_unique<ParabolicDepthEstimator>());
  DccLensMapper mapper;
  mapper.init(dcc);

  AfController ctrl(pipeline, mapper, actuator, tun);
  ctrl.trigger();

  AfRequest req{{Roi{0, 0, 256, 128}}};
  int frames = 0;
  for (; frames < 100; ++frames) {
    auto in = source.capture(req);
    ctrl.onFrame(req, in);
    if (ctrl.state() == AfState::kFocused || ctrl.state() == AfState::kFailed) break;
  }
  return {ctrl.state(), frames + 1,
          std::abs(world.currentStep() - world.inFocusStep())};
}

}  // namespace

TEST(ClosedLoop, MidDistanceConverges) {
  auto r = runScenario(2000.0, 300, 0.01f);  // in-focus=175, d0=-2.5
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);  // 0.25 disparity*50 steps + 量測 sub-pixel 誤差
  EXPECT_LT(r.frames_used, 40);
}

TEST(ClosedLoop, NearDistanceConverges) {
  auto r = runScenario(300.0, 300, 0.01f);  // in-focus=600, d0=+6
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);
}

TEST(ClosedLoop, FarDistanceLargeDefocusConverges) {
  auto r = runScenario(5000.0, 600, 0.01f);  // in-focus=130, d0=-9.4
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.final_error_steps, 20);
}

TEST(ClosedLoop, AlreadyInFocusFinishesImmediately) {
  auto r = runScenario(2000.0, 175, 0.005f);  // 初始即合焦
  EXPECT_EQ(r.final_state, AfState::kFocused);
  EXPECT_LE(r.frames_used, 5);
}
```

- [ ] **Step 2: 跑整合測試確認通過（元件已齊，此測試應直接綠）**

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: 全部 PASS（含 ClosedLoop 4 條）。若 ClosedLoop 有紅，回頭修對應元件——這是整合驗收，不改測試遷就實作。

- [ ] **Step 3: 建立 CLI `apps/CMakeLists.txt` 與 `apps/pdaf_cli/main.cpp`**

`apps/CMakeLists.txt`：

```cmake
add_executable(pdaf_cli pdaf_cli/main.cpp)
target_link_libraries(pdaf_cli PRIVATE pdaf)
```

根 `CMakeLists.txt` 的 `enable_testing()` 前加一行 `add_subdirectory(apps)`。

`apps/pdaf_cli/main.cpp`：

```cpp
#include <algo/dcc_lens_mapper.h>
#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>
#include <control/pdaf_pipeline.h>
#include <control/run_logger.h>
#include <pdaf/control/af_config.h>
#include <pdaf/control/af_controller.h>
#include <replay/null_lens_actuator.h>
#include <replay/replay_pd_data_source.h>
#include <sim/sim_world.h>

#include <cstring>
#include <iostream>
#include <memory>

using namespace pdaf;

namespace {

struct Args {
  std::string config = "config/default.json";
  std::string mode;     // 空 = 用 config 的值
  std::string out;      // 空 = 用 config 的 log_dir
  int max_frames = 200;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
      return argv[++i];
    };
    if (!std::strcmp(argv[i], "--config")) a.config = need("--config");
    else if (!std::strcmp(argv[i], "--mode")) a.mode = need("--mode");
    else if (!std::strcmp(argv[i], "--out")) a.out = need("--out");
    else if (!std::strcmp(argv[i], "--max-frames")) a.max_frames = std::atoi(need("--max-frames"));
    else throw std::runtime_error(std::string("unknown argument: ") + argv[i]);
  }
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    AfConfig cfg = AfConfig::loadFromFile(args.config);
    if (!args.mode.empty()) cfg.system.mode = args.mode;
    if (!args.out.empty()) cfg.system.log_dir = args.out;

    // 組裝：依 mode 選 HAL 實作，注入三模塊（demo 的 DI 就在這裡）
    std::unique_ptr<SimWorld> world;
    std::unique_ptr<IPdDataSource> source;
    std::unique_ptr<ILensActuator> actuator;
    if (cfg.system.mode == "sim") {
      world = std::make_unique<SimWorld>(cfg.sensor, cfg.system.sim, cfg.calibration.dcc);
      source = std::make_unique<SimPdDataSource>(*world);
      actuator = std::make_unique<SimLensActuator>(*world);
    } else {
      source = std::make_unique<ReplayPdDataSource>(cfg.system.replay_dir);
      actuator = std::make_unique<NullLensActuator>(0);
    }

    auto m1 = std::make_unique<SadCostEngine>();
    m1->init(cfg.calibration.lrc, cfg.sensor.pattern,
             cfg.tuning.shift_min, cfg.tuning.shift_max);
    PdafPipeline pipeline(std::move(m1), std::make_unique<ParabolicDepthEstimator>());
    DccLensMapper mapper;
    mapper.init(cfg.calibration.dcc);
    AfController ctrl(pipeline, mapper, *actuator, cfg.tuning);
    RunLogger logger(cfg.system.log_dir);

    ctrl.trigger();
    AfRequest req{{cfg.sensor.default_roi}};
    int frames = 0;
    for (; frames < args.max_frames; ++frames) {
      const PdInput in = source->capture(req);
      const AfFrameLog log = ctrl.onFrame(req, in);
      logger.logFrame(log, world ? world->groundTruthDisparity() : 0.f);
      if (ctrl.state() == AfState::kFocused || ctrl.state() == AfState::kFailed) break;
    }

    const int final_step = actuator->getStatus().current_step;
    const int gt = world ? world->inFocusStep() : -1;
    logger.writeSummary(ctrl.state(), frames + 1, final_step, gt);

    std::cout << "final_state: " << toString(ctrl.state())
              << "  frames: " << frames + 1 << "  final_step: " << final_step;
    if (world) std::cout << "  in_focus_step: " << gt;
    std::cout << "\nlogs: " << cfg.system.log_dir << "/frames.csv\n";
    return ctrl.state() == AfState::kFocused ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
```

- [ ] **Step 4: 建置並手動驗證 CLI**

Run: `cmake --build build && ./build/apps/pdaf_cli --config config/default.json --out /tmp/pdaf_run`
Expected: stdout 印出 `final_state: FOCUSED  frames: <N>  final_step: ~175  in_focus_step: 175`；`/tmp/pdaf_run/frames.csv` 與 `summary.txt` 存在，exit code 0

Run: `ctest --test-dir build --output-on-failure`
Expected: 全部 PASS

- [ ] **Step 5: Commit**

```bash
git add apps/ tests/ CMakeLists.txt
git commit -m "feat: pdaf_cli（組裝+frame loop+輸出）與閉環整合測試"
```

---

## 完成後

全部 task 完成後：
1. 跑一次完整驗證：`rm -rf build && cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure`
2. 用 superpowers:finishing-a-development-branch 技能收尾
