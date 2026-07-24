# M2 cost sequence 視覺化驗證工具 實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立 M2（ParabolicDepthEstimator）的 cost sequence 視覺化驗證工具：估測器透出中間值、sim 寫出實體 cost 檔、桌面 GUI（ImGui+ImPlot）行程內跑真實估測並繪圖標註。

**Architecture:** 估測器加 `estimateTraced()`（`estimate()` 改薄包裝，數學一份）；cost 檔沿用 replay `hw_costs` 格式 + 選用真值；`pdaf_cli --dump-costs` 產檔；`pdaf_cost_viz`（opt-in GUI）連 `libpdaf` 直接跑真實估測。詳見 spec：`docs/superpowers/specs/2026-07-24-cost-viz-tool-design.md`。

**Tech Stack:** C++17、CMake、GoogleTest、nlohmann/json（已 vendored）；GUI 用 Dear ImGui + ImPlot + GLFW + OpenGL3（FetchContent，opt-in）。

## Global Constraints

- C++17，所有程式碼在 `namespace pdaf`
- 程式碼風格 `.clang-format`（Allman braces、2-space、`ColumnLimit: 0`、`InsertBraces: true`）——pre-commit hook 與 CI 會擋不符格式的 commit
- 執行期估測器不丟例外，退化情況以旗標/invalid 呈現
- 單一測試 binary `pdaf_tests`（新測試檔加入 `tests/CMakeLists.txt` 的 `add_executable`，不新增 target）
- GUI 為 opt-in：`option(PDAF_BUILD_GUI OFF)`，核心 `libpdaf`／`pdaf_tests`／`pdaf_cli` 維持零 GUI 相依
- 測試指令：`cmake --preset default && cmake --build --preset default && ctest --preset default`
- 每個 task 結尾 commit，conventional commits（feat/test/build）

---

### Task 1: 估測器透出中間值（DepthEstimateTrace + estimateTraced）

**Files:**
- Modify: `src/algo/parabolic_depth_estimator.h`（加 `DepthEstimateTrace` 與 `estimateTraced` 宣告）
- Modify: `src/algo/parabolic_depth_estimator.cpp`（把 `estimate()` 的數學搬進 `estimateTraced()`，`estimate()` 改薄包裝）
- Test: `tests/test_depth_estimator.cpp`（既有檔，追加 trace 測試）

**Interfaces:**
- Consumes: `CostSequence`、`DepthEstimate`（`include/pdaf/types.h`，已存在）
- Produces:
```cpp
struct DepthEstimateTrace {
  DepthEstimate result;
  bool degenerate_no_samples = false; bool degenerate_flat = false; bool boundary = false;
  std::size_t mi = 0; float cmin = 0.f; float mean = 0.f; float depth = 0.f;
  std::size_t basin_lo = 0, basin_hi = 0; float second = 0.f; float unamb = 0.f;
  float c_m1 = 0.f, c_0 = 0.f, c_p1 = 0.f; float sharp = 0.f; float delta = 0.f;
};
// ParabolicDepthEstimator::estimateTraced(const CostSequence&) const -> DepthEstimateTrace
// ParabolicDepthEstimator::estimate(...) 改為 return estimateTraced(cost).result;
```

- [ ] **Step 1: 寫失敗測試**（追加到 `tests/test_depth_estimator.cpp` 末端）

```cpp
TEST(DepthEstimatorTrace, ResultMatchesEstimate)
{
  ParabolicDepthEstimator m2;
  const std::vector<CostSequence> cases = {
      makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4}),               // 一般
      makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}),           // 邊界
      makeCost(-4, std::vector<float>(9, 5.f)),                   // 平（低 depth）
      makeCost(-4, {8, 1.2f, 8, 3, 1, 3, 8, 1.3f, 8}),           // 雙峰
      makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0),              // no samples
  };
  for (const auto& c : cases)
  {
    const DepthEstimate a = m2.estimate(c);
    const DepthEstimate b = m2.estimateTraced(c).result;
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_FLOAT_EQ(a.disparity, b.disparity);
    EXPECT_FLOAT_EQ(a.confidence, b.confidence);
  }
}

TEST(DepthEstimatorTrace, GoldenUnimodal)
{
  // {8,6,4,2,1,0.2,1,2,4} shift_min -4：full basin、無競爭谷
  auto t = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {8, 6, 4, 2, 1, 0.2f, 1, 2, 4}));
  EXPECT_FALSE(t.degenerate_no_samples);
  EXPECT_FALSE(t.degenerate_flat);
  EXPECT_FALSE(t.boundary);
  EXPECT_EQ(t.mi, 5u);
  EXPECT_FLOAT_EQ(t.cmin, 0.2f);
  EXPECT_NEAR(t.mean, 3.1333f, 1e-3f);
  EXPECT_NEAR(t.depth, 0.9362f, 1e-3f);
  EXPECT_EQ(t.basin_lo, 0u);
  EXPECT_EQ(t.basin_hi, 8u);
  EXPECT_TRUE(std::isinf(t.second)); // 無競爭谷
  EXPECT_FLOAT_EQ(t.unamb, 1.f);
  EXPECT_FLOAT_EQ(t.c_m1, 1.f);
  EXPECT_FLOAT_EQ(t.c_0, 0.2f);
  EXPECT_FLOAT_EQ(t.c_p1, 1.f);
  EXPECT_NEAR(t.sharp, 0.8f, 1e-4f);
  EXPECT_NEAR(t.delta, 0.f, 1e-4f);
  EXPECT_NEAR(t.result.disparity, 1.f, 1e-4f);
  EXPECT_NEAR(t.result.confidence, 0.8426f, 1e-3f);
  EXPECT_TRUE(t.result.valid);
}

TEST(DepthEstimatorTrace, GoldenBimodalBasinAndSecond)
{
  // {8,1.2,8,3,1,3,8,1.3,8} shift_min -4：主谷 basin [2,6]、競爭谷 second=1.2
  auto t = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {8, 1.2f, 8, 3, 1, 3, 8, 1.3f, 8}));
  EXPECT_EQ(t.mi, 4u);
  EXPECT_EQ(t.basin_lo, 2u);
  EXPECT_EQ(t.basin_hi, 6u);
  EXPECT_FLOAT_EQ(t.second, 1.2f);
  EXPECT_NEAR(t.unamb, 0.0554f, 1e-3f);
  EXPECT_NEAR(t.result.disparity, 0.f, 1e-4f);
  EXPECT_LT(t.result.confidence, 0.15f);
}

TEST(DepthEstimatorTrace, DegenerateFlagsSet)
{
  auto ns = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 0));
  EXPECT_TRUE(ns.degenerate_no_samples);
  EXPECT_FALSE(ns.result.valid);
  auto flat = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, std::vector<float>(9, 0.f)));
  EXPECT_TRUE(flat.degenerate_flat);
  EXPECT_FALSE(flat.result.valid);
  auto edge = ParabolicDepthEstimator{}.estimateTraced(makeCost(-4, {0.2f, 1, 2, 4, 6, 8, 10, 12, 14}));
  EXPECT_TRUE(edge.boundary);
  EXPECT_TRUE(edge.result.valid);
}
```

需要在測試檔頂端補 `#include <cmath>`（`std::isinf`）——若尚未 include。

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build --preset default`
Expected: FAIL — `estimateTraced` / `DepthEstimateTrace` 未宣告

- [ ] **Step 3: 實作** — `src/algo/parabolic_depth_estimator.h`

```cpp
#pragma once
#include <pdaf/algo/depth_estimator.h>

#include <cstddef>

namespace pdaf
{
// estimateTraced() 記錄的中間值，供視覺化/驗證工具標示（與生產路徑同一份計算）
struct DepthEstimateTrace
{
  DepthEstimate result;

  bool degenerate_no_samples = false; // v.size()<3 或 valid_samples<=0
  bool degenerate_flat = false;       // mean < 1e-6
  bool boundary = false;              // 最小點在搜尋邊界

  std::size_t mi = 0;
  float cmin = 0.f;
  float mean = 0.f;
  float depth = 0.f;
  std::size_t basin_lo = 0, basin_hi = 0;
  float second = 0.f; // basin 外最低成本；無競爭谷時為 +inf
  float unamb = 0.f;

  // interior（非邊界）才有效
  float c_m1 = 0.f, c_0 = 0.f, c_p1 = 0.f;
  float sharp = 0.f;
  float delta = 0.f;
};

// 參考實作：cost 極小值 + 三點拋物線內插；confidence = depth × unamb × (0.5+0.5·sharp)
class ParabolicDepthEstimator : public IDepthEstimator
{
public:
  DepthEstimate estimate(const CostSequence& cost) override;
  DepthEstimateTrace estimateTraced(const CostSequence& cost) const;
};
} // namespace pdaf
```

`src/algo/parabolic_depth_estimator.cpp`（整檔取代）：

```cpp
#include "algo/parabolic_depth_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace pdaf
{

DepthEstimateTrace ParabolicDepthEstimator::estimateTraced(const CostSequence& cost) const
{
  DepthEstimateTrace t;
  const auto& v = cost.costs;
  if (v.size() < 3 || cost.valid_samples <= 0)
  {
    t.degenerate_no_samples = true;
    return t;
  }

  const size_t n = v.size();
  t.mi = std::min_element(v.begin(), v.end()) - v.begin();
  t.mean = std::accumulate(v.begin(), v.end(), 0.f) / n;
  if (t.mean < 1e-6f)
  {
    t.degenerate_flat = true; // 全平：無紋理可匹配
    return t;
  }
  t.cmin = v[t.mi];

  // depth：谷底相對整體平均的深度，除以 mean 使其對場景對比尺度不變
  t.depth = std::clamp(1.f - t.cmin / t.mean, 0.f, 1.f);

  // 次低點檢查：排除主谷「整段單調 basin」，只有被山脊隔開的競爭谷才扣分
  size_t lo = t.mi;
  while (lo > 0 && v[lo - 1] >= v[lo])
  {
    --lo;
  }
  size_t hi = t.mi;
  while (hi + 1 < n && v[hi + 1] >= v[hi])
  {
    ++hi;
  }
  t.basin_lo = lo;
  t.basin_hi = hi;
  t.second = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < n; ++i)
  {
    if (i < lo || i > hi)
    {
      t.second = std::min(t.second, v[i]);
    }
  }
  const float depth_abs = t.mean - t.cmin;
  t.unamb = (std::isinf(t.second) || depth_abs < 1e-9f)
                ? 1.f
                : std::clamp((t.second - t.cmin) / depth_abs, 0.f, 1.f);

  if (t.mi == 0 || t.mi + 1 == n)
  {
    // 邊界：真值可能在範圍外，不內插；曲率未定義，信心對折並套用歧義因子
    t.boundary = true;
    t.result = {static_cast<float>(cost.shift_min + static_cast<int>(t.mi)), t.depth * t.unamb * 0.5f, true};
    return t;
  }
  t.c_m1 = v[t.mi - 1];
  t.c_0 = v[t.mi];
  t.c_p1 = v[t.mi + 1];
  const float denom = t.c_m1 - 2.f * t.c_0 + t.c_p1;
  t.delta = denom > 1e-9f ? 0.5f * (t.c_m1 - t.c_p1) / denom : 0.f;

  // 曲率／尖銳度：正規化二階差分（= 局部鄰點對比）
  const float side = t.c_m1 + t.c_p1;
  t.sharp = side > 1e-9f ? std::clamp(1.f - 2.f * t.c_0 / side, 0.f, 1.f) : 0.f;

  // 組合：歧義為硬 gate、曲率為輕度折扣（最多對折）
  const float conf = std::clamp(t.depth * t.unamb * (0.5f + 0.5f * t.sharp), 0.f, 1.f);
  t.result = {static_cast<float>(cost.shift_min + static_cast<int>(t.mi)) + t.delta, conf, true};
  return t;
}

DepthEstimate ParabolicDepthEstimator::estimate(const CostSequence& cost)
{
  return estimateTraced(cost).result;
}

} // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build --preset default && ctest --preset default`
Expected: 全部 PASS（含既有 DepthEstimator 8 條 + 新 DepthEstimatorTrace 4 條）

- [ ] **Step 5: Commit**

```bash
git add src/algo/parabolic_depth_estimator.h src/algo/parabolic_depth_estimator.cpp tests/test_depth_estimator.cpp
git commit -m "feat: ParabolicDepthEstimator 加 estimateTraced 透出中間值"
```

---

### Task 2: cost frame 寫檔（writeCostFrame）

**Files:**
- Create: `src/replay/cost_dump.h`, `src/replay/cost_dump.cpp`
- Test: `tests/test_cost_dump.cpp`
- Modify: 根 `CMakeLists.txt`（`target_sources` 加 `src/replay/cost_dump.cpp`）、`tests/CMakeLists.txt`（`add_executable` 加 `test_cost_dump.cpp`）

**Interfaces:**
- Consumes: `FrameMeta`、`CostSequence`（types.h）；`nlohmann/json`
- Produces:
```cpp
// 把一個 frame 的 cost sequences 寫成 replay hw_costs 格式：<dir>/frame_%04d.json（用 meta.frame_id）。
// ground_truth 非空（size 需等於 costs.size()）時附上 ground_truth_disparity。
void writeCostFrame(const std::string& dir, const FrameMeta& meta,
                    const std::vector<CostSequence>& costs,
                    const std::vector<float>& ground_truth);
```

- [ ] **Step 1: 寫失敗測試 `tests/test_cost_dump.cpp`**

```cpp
#include <replay/cost_dump.h>
#include <replay/replay_pd_data_source.h>

#include <nlohmann/json.hpp>
#include <gtest/gtest.h>

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

  const auto file = dir / "frame_0003.json";
  ASSERT_TRUE(fs::exists(file));

  std::ifstream f(file);
  std::stringstream ss;
  ss << f.rdbuf();
  auto j = nlohmann::json::parse(ss.str());
  EXPECT_EQ(j["meta"]["frame_id"].get<uint64_t>(), 3u);
  EXPECT_EQ(j["meta"]["lens_step_at_exposure"].get<int>(), 300);
  EXPECT_EQ(j["hw_costs"][0]["shift_min"].get<int>(), -2);
  EXPECT_EQ(j["hw_costs"][0]["costs"].size(), 5u);
  EXPECT_EQ(j["hw_costs"][0]["valid_samples"].get<int>(), 40);
  EXPECT_FLOAT_EQ(j["ground_truth_disparity"][0].get<float>(), -2.5f);

  // 同一份檔案能被 replay 讀回（向下相容）
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
```

- [ ] **Step 2: 跑測試確認編譯失敗**

Run: `cmake --build --preset default`
Expected: FAIL — `replay/cost_dump.h` 找不到

- [ ] **Step 3: 實作** — `src/replay/cost_dump.h`

```cpp
#pragma once
#include <pdaf/types.h>

#include <string>
#include <vector>

namespace pdaf
{
// 把一個 frame 的 cost sequences 寫成 replay hw_costs 格式 JSON（<dir>/frame_%04d.json）。
// ground_truth 非空時附上 ground_truth_disparity（size 需等於 costs.size()）。
void writeCostFrame(const std::string& dir, const FrameMeta& meta,
                    const std::vector<CostSequence>& costs,
                    const std::vector<float>& ground_truth);
} // namespace pdaf
```

`src/replay/cost_dump.cpp`：

```cpp
#include "replay/cost_dump.h"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace pdaf
{

void writeCostFrame(const std::string& dir, const FrameMeta& meta,
                    const std::vector<CostSequence>& costs,
                    const std::vector<float>& ground_truth)
{
  std::filesystem::create_directories(dir);

  nlohmann::json j;
  j["meta"] = {{"frame_id", meta.frame_id},
               {"timestamp_ms", meta.timestamp_ms},
               {"lens_step_at_exposure", meta.lens_step_at_exposure}};
  for (const auto& c : costs)
  {
    j["hw_costs"].push_back({{"shift_min", c.shift_min}, {"costs", c.costs}, {"valid_samples", c.valid_samples}});
  }
  if (!ground_truth.empty())
  {
    j["ground_truth_disparity"] = ground_truth;
  }

  char name[32];
  std::snprintf(name, sizeof(name), "frame_%04llu.json", static_cast<unsigned long long>(meta.frame_id));
  std::ofstream(std::filesystem::path(dir) / name) << j.dump(2) << '\n';
}

} // namespace pdaf
```

- [ ] **Step 4: 跑測試確認通過**

Run: `cmake --build --preset default && ctest --preset default`
Expected: 全部 PASS（含 CostDump 2 條）

- [ ] **Step 5: Commit**

```bash
git add src/replay/cost_dump.h src/replay/cost_dump.cpp tests/test_cost_dump.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: writeCostFrame 寫出 replay 格式 cost frame（+選用真值）"
```

---

### Task 3: pdaf_cli --dump-costs

**Files:**
- Modify: `apps/pdaf_cli/main.cpp`（加 `--dump-costs` 旗標與 dump 邏輯）
- Test: `tests/test_cli_dump.cpp`
- Modify: `tests/CMakeLists.txt`（加 `test_cli_dump.cpp` 與 `PDAF_CLI_BIN` 定義）

**Interfaces:**
- Consumes: `writeCostFrame`（Task 2）、`SadCostEngine`、`SimWorld`
- Produces: `pdaf_cli --dump-costs <dir>`（sim 模式每 frame 寫 `<dir>/frame_%04d.json`，含真值）

- [ ] **Step 1: 寫失敗測試 `tests/test_cli_dump.cpp`**

```cpp
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
```

`tests/CMakeLists.txt` 的 `target_compile_definitions` 追加 CLI 路徑（`PDAF_SOURCE_DIR` 應已存在）：

```cmake
target_compile_definitions(pdaf_tests PRIVATE PDAF_CLI_BIN="$<TARGET_FILE:pdaf_cli>")
```
並確保 `pdaf_tests` 在 `pdaf_cli` 之後可取得其路徑（同一 CMake 專案，`$<TARGET_FILE:>` 於 generator 期解析，無需顯式相依；若測試執行早於 CLI 建置，加 `add_dependencies(pdaf_tests pdaf_cli)`）。

- [ ] **Step 2: 跑測試確認編譯/執行失敗**

Run: `cmake --preset default && cmake --build --preset default`
Expected: 編譯失敗（`PDAF_CLI_BIN` 未定義）或測試失敗（`--dump-costs` 未實作 → CLI 回非零 `unknown argument`）

- [ ] **Step 3: 實作** — 修改 `apps/pdaf_cli/main.cpp`

在檔頭 include 加：
```cpp
#include <replay/cost_dump.h>
```
`Args` 結構加欄位：
```cpp
  std::string dump_costs; // 空 = 不 dump
```
`parseArgs` 的 `else if` 鏈加一支（在 `--max-frames` 之後、`else` 之前）：
```cpp
    else if (!std::strcmp(argv[i], "--dump-costs"))
    {
      a.dump_costs = need("--dump-costs");
    }
```
組裝完（`RunLogger logger(...)` 之後）加一個 dump 專用 M1（sim 模式才建）：
```cpp
    std::unique_ptr<SadCostEngine> dump_m1;
    if (!args.dump_costs.empty() && cfg.system.mode == "sim")
    {
      dump_m1 = std::make_unique<SadCostEngine>();
      dump_m1->init(cfg.calibration.lrc, cfg.sensor.pattern, cfg.tuning.shift_min, cfg.tuning.shift_max);
    }
    else if (!args.dump_costs.empty())
    {
      std::cerr << "warning: --dump-costs 只在 sim 模式有作用，已略過\n";
    }
```
frame loop 內（`logger.logFrame(...)` 之後、收斂判斷之前）加：
```cpp
      if (dump_m1 && in.raw)
      {
        auto costs = dump_m1->compute(*in.raw);
        std::vector<float> gt(costs.size(), world->groundTruthDisparity());
        writeCostFrame(args.dump_costs, in.meta, costs, gt);
      }
```

- [ ] **Step 4: 跑測試確認通過 + 手動驗證**

Run: `cmake --build --preset default && ctest --preset default`
Expected: 全部 PASS（含 CliDump）

Run: `./build/apps/pdaf_cli --config config/default.json --out /tmp/o --dump-costs /tmp/costs --max-frames 5 && ls /tmp/costs`
Expected: `frame_0000.json` … 數個檔；任取一個 `cat` 應見 `hw_costs` 與 `ground_truth_disparity`

- [ ] **Step 5: Commit**

```bash
git add apps/pdaf_cli/main.cpp tests/test_cli_dump.cpp tests/CMakeLists.txt
git commit -m "feat: pdaf_cli --dump-costs 寫出 sim 的 cost frame"
```

---

### Task 4: opt-in ImGui+ImPlot 桌面 GUI（pdaf_cost_viz）

**Files:**
- Create: `tools/cost_viz/main.cpp`, `tools/CMakeLists.txt`
- Modify: 根 `CMakeLists.txt`（加 `option(PDAF_BUILD_GUI OFF)` 與條件 `add_subdirectory(tools)`）

**Interfaces:**
- Consumes: `ParabolicDepthEstimator::estimateTraced`（Task 1）、cost frame JSON（Task 2/3 格式）、`nlohmann/json`
- Produces: `pdaf_cost_viz` 執行檔（`-DPDAF_BUILD_GUI=ON` 才建）；用法 `pdaf_cost_viz [<檔案或目錄>]`

- [ ] **Step 1: 根 `CMakeLists.txt` 加 opt-in 開關**

在 `add_subdirectory(apps)` 之後、`enable_testing()` 之前加：
```cmake
option(PDAF_BUILD_GUI "Build ImGui/ImPlot desktop tools" OFF)
if(PDAF_BUILD_GUI)
  add_subdirectory(tools)
endif()
```

- [ ] **Step 2: `tools/CMakeLists.txt`（FetchContent 相依）**

```cmake
include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.91.5)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(implot GIT_REPOSITORY https://github.com/epezent/implot.git GIT_TAG v0.16)
FetchContent_MakeAvailable(implot)

find_package(OpenGL REQUIRED)

add_executable(pdaf_cost_viz
  cost_viz/main.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp)
target_include_directories(pdaf_cost_viz PRIVATE
  ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${implot_SOURCE_DIR})
target_link_libraries(pdaf_cost_viz PRIVATE pdaf glfw OpenGL::GL)
```

- [ ] **Step 3: `tools/cost_viz/main.cpp`（ImGui+ImPlot app）**

```cpp
#include <algo/parabolic_depth_estimator.h>
#include <pdaf/types.h>

#include <nlohmann/json.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <GLFW/glfw3.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace pdaf;
namespace fs = std::filesystem;

namespace
{

struct LoadedFrame
{
  FrameMeta meta;
  std::vector<CostSequence> costs;
  std::vector<float> gt; // 可能為空
};

struct App
{
  char path_buf[512] = "";
  std::string error;
  std::vector<LoadedFrame> frames;
  int frame_idx = 0;
  int roi_idx = 0;
};

double ys_max(const std::vector<double>& v)
{
  double m = 0;
  for (double x : v)
  {
    m = x > m ? x : m;
  }
  return m * 1.05;
}

std::optional<LoadedFrame> parseFrameFile(const fs::path& file, std::string& err)
{
  std::ifstream f(file);
  if (!f)
  {
    err = "無法開啟 " + file.string();
    return std::nullopt;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  try
  {
    auto j = nlohmann::json::parse(ss.str());
    LoadedFrame lf;
    const auto& m = j.at("meta");
    lf.meta.frame_id = m.at("frame_id").get<uint64_t>();
    lf.meta.timestamp_ms = m.at("timestamp_ms").get<double>();
    lf.meta.lens_step_at_exposure = m.at("lens_step_at_exposure").get<int>();
    for (const auto& c : j.at("hw_costs"))
    {
      lf.costs.push_back({c.at("shift_min").get<int>(), c.at("costs").get<std::vector<float>>(),
                          c.at("valid_samples").get<int>()});
    }
    if (j.contains("ground_truth_disparity"))
    {
      lf.gt = j["ground_truth_disparity"].get<std::vector<float>>();
    }
    return lf;
  }
  catch (const std::exception& e)
  {
    err = std::string("解析失敗 ") + file.filename().string() + ": " + e.what();
    return std::nullopt;
  }
}

void loadPath(App& app, const std::string& path)
{
  app.frames.clear();
  app.frame_idx = 0;
  app.roi_idx = 0;
  app.error.clear();
  if (path.empty() || !fs::exists(path))
  {
    app.error = "路徑不存在：" + path;
    return;
  }
  std::vector<fs::path> files;
  if (fs::is_directory(path))
  {
    for (int i = 0;; ++i)
    {
      char name[32];
      std::snprintf(name, sizeof(name), "frame_%04d.json", i);
      const fs::path p = fs::path(path) / name;
      if (!fs::exists(p))
      {
        break;
      }
      files.push_back(p);
    }
    if (files.empty())
    {
      app.error = "目錄下無 frame_0000.json：" + path;
      return;
    }
  }
  else
  {
    files.push_back(path);
  }
  for (const auto& file : files)
  {
    std::string err;
    if (auto lf = parseFrameFile(file, err))
    {
      app.frames.push_back(std::move(*lf));
    }
    else
    {
      app.error = err;
    }
  }
}

void drawPlot(const CostSequence& c, const DepthEstimateTrace& t, const float* gt)
{
  const int nn = static_cast<int>(c.costs.size());
  std::vector<double> xs(nn), ys(nn);
  for (int i = 0; i < nn; ++i)
  {
    xs[i] = c.shift_min + i;
    ys[i] = c.costs[i];
  }
  if (ImPlot::BeginPlot("cost sequence", ImVec2(-1, -1)))
  {
    ImPlot::SetupAxes("shift s", "SAD cost");
    // basin 陰影
    if (!t.degenerate_no_samples && !t.degenerate_flat)
    {
      const double bx0 = c.shift_min + static_cast<int>(t.basin_lo);
      const double bx1 = c.shift_min + static_cast<int>(t.basin_hi);
      double bxs[2] = {bx0, bx1};
      double top[2] = {ys_max(ys), ys_max(ys)};
      double bot[2] = {0, 0};
      ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.12f);
      ImPlot::PlotShaded("basin", bxs, bot, top, 2);
      ImPlot::PopStyleVar();
    }
    // cost 曲線 + 點
    ImPlot::PlotLine("cost", xs.data(), ys.data(), nn);
    ImPlot::PlotScatter("samples", xs.data(), ys.data(), nn);
    // mean 水平線
    if (!t.degenerate_no_samples)
    {
      double mean = t.mean;
      ImPlot::PlotInfLines("mean", &mean, 1, ImPlotInfLinesFlags_Horizontal);
    }
    // cmin
    if (!t.degenerate_no_samples && !t.degenerate_flat)
    {
      double cx = c.shift_min + static_cast<int>(t.mi);
      double cy = t.cmin;
      ImPlot::PlotScatter("cmin", &cx, &cy, 1);
    }
    // 次低點（有競爭谷）
    if (!std::isinf(t.second))
    {
      // 找 second 對應的 x（basin 外的最低點）
      for (int i = 0; i < nn; ++i)
      {
        if ((i < static_cast<int>(t.basin_lo) || i > static_cast<int>(t.basin_hi)) && c.costs[i] == t.second)
        {
          double sx = xs[i], sy = ys[i];
          ImPlot::PlotScatter("second", &sx, &sy, 1);
          break;
        }
      }
    }
    // disparity 垂直線
    if (t.result.valid)
    {
      double dx = t.result.disparity;
      ImPlot::PlotInfLines("disparity", &dx, 1);
    }
    // 真值
    if (gt)
    {
      double g = *gt;
      ImPlot::PlotInfLines("ground truth", &g, 1);
    }
    ImPlot::EndPlot();
  }
}

} // namespace

int main(int argc, char** argv)
{
  App app;
  if (argc > 1)
  {
    std::snprintf(app.path_buf, sizeof(app.path_buf), "%s", argv[1]);
    loadPath(app, argv[1]);
  }

  if (!glfwInit())
  {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow* win = glfwCreateWindow(1100, 720, "pdaf_cost_viz — M2 cost sequence", nullptr, nullptr);
  if (!win)
  {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  ParabolicDepthEstimator m2;

  while (!glfwWindowShouldClose(win))
  {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("main", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImGui::SetNextItemWidth(600);
    ImGui::InputText("路徑（檔案或目錄）", app.path_buf, sizeof(app.path_buf));
    ImGui::SameLine();
    if (ImGui::Button("載入"))
    {
      loadPath(app, app.path_buf);
    }
    if (!app.error.empty())
    {
      ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", app.error.c_str());
    }

    if (!app.frames.empty())
    {
      app.frame_idx = std::min(app.frame_idx, static_cast<int>(app.frames.size()) - 1);
      ImGui::SliderInt("frame", &app.frame_idx, 0, static_cast<int>(app.frames.size()) - 1);
      const LoadedFrame& lf = app.frames[app.frame_idx];
      if (!lf.costs.empty())
      {
        app.roi_idx = std::min(app.roi_idx, static_cast<int>(lf.costs.size()) - 1);
        if (lf.costs.size() > 1)
        {
          ImGui::SliderInt("ROI", &app.roi_idx, 0, static_cast<int>(lf.costs.size()) - 1);
        }
        const CostSequence& c = lf.costs[app.roi_idx];
        const DepthEstimateTrace t = m2.estimateTraced(c);
        const float* gt = (app.roi_idx < static_cast<int>(lf.gt.size())) ? &lf.gt[app.roi_idx] : nullptr;

        // 左：數據面板；右：圖
        ImGui::Columns(2, nullptr, true);
        ImGui::SetColumnWidth(0, 340);
        ImGui::Text("frame_id: %llu", static_cast<unsigned long long>(lf.meta.frame_id));
        ImGui::Text("lens@exposure: %d", lf.meta.lens_step_at_exposure);
        ImGui::Text("shift: [%d, %d]  valid: %d", c.shift_min,
                    c.shift_min + static_cast<int>(c.costs.size()) - 1, c.valid_samples);
        ImGui::Separator();
        if (t.degenerate_no_samples)
        {
          ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: no samples → invalid");
        }
        else if (t.degenerate_flat)
        {
          ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: flat (無紋理) → invalid");
        }
        else
        {
          ImGui::Text("mi: %zu   cmin: %.4f   mean: %.4f", t.mi, t.cmin, t.mean);
          ImGui::Text("depth: %.3f", t.depth);
          ImGui::Text("basin: [%zu, %zu]", t.basin_lo, t.basin_hi);
          ImGui::Text("second: %s", std::isinf(t.second) ? "inf (無競爭谷)" : std::to_string(t.second).c_str());
          ImGui::Text("unamb: %.3f", t.unamb);
          if (t.boundary)
          {
            ImGui::TextColored(ImVec4(1, 0.6f, 0, 1), "boundary → 不內插、conf×0.5");
          }
          else
          {
            ImGui::Text("c-1,c0,c+1: %.4f, %.4f, %.4f", t.c_m1, t.c_0, t.c_p1);
            ImGui::Text("sharp: %.3f   delta: %.4f", t.sharp, t.delta);
          }
          ImGui::Separator();
          ImGui::Text("disparity: %.4f", t.result.disparity);
          ImGui::Text("confidence: %.4f", t.result.confidence);
          if (gt)
          {
            ImGui::Text("ground truth: %.4f", *gt);
            ImGui::Text("error: %.4f", std::abs(t.result.disparity - *gt));
          }
        }
        ImGui::NextColumn();
        drawPlot(c, t, gt);
        ImGui::Columns(1);
      }
    }
    ImGui::End();

    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(win, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
```

注意：`ys_max` 已定義於 `drawPlot` 之上（見上方程式碼），順序正確。需要的 header：`<algorithm>`（`std::min`）、`<cmath>`（`std::isinf`、`std::abs`）、`<cstdint>`（`uint64_t`）——若編譯器未透過其他 header 帶入，補上。

- [ ] **Step 4: 建置（opt-in）並驗證編譯連結**

Run: `cmake --preset default -DPDAF_BUILD_GUI=ON && cmake --build --preset default`
Expected: `pdaf_cost_viz` 成功編譯連結（FetchContent 拉 glfw/imgui/implot；Linux 需 OpenGL/X11 dev 標頭，缺則 configure 會報明確錯誤）。

驗證核心不受影響：`cmake --preset default && cmake --build --preset default && ctest --preset default` 仍全綠、且**不**建 GUI。

- [ ] **Step 5: 手動驗證（有顯示環境時）**

```bash
./build/apps/pdaf_cli --config config/default.json --out /tmp/o --dump-costs /tmp/costs --max-frames 5
./build/tools/pdaf_cost_viz /tmp/costs
```
Expected: 開窗、frame slider 可切換、圖上見 cost 曲線與 cmin/mean/basin/disparity 標示、面板列出中間值。無顯示環境（CI/headless）僅需通過 Step 4 的編譯連結。

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tools/CMakeLists.txt tools/cost_viz/main.cpp
git commit -m "feat: pdaf_cost_viz 桌面 GUI（ImGui+ImPlot，opt-in PDAF_BUILD_GUI）"
```

---

## 完成後

1. 核心驗證（不含 GUI）：`rm -rf build && cmake --preset default && cmake --build --preset default && ctest --preset default` 全綠
2. GUI 建置驗證：`cmake --preset default -DPDAF_BUILD_GUI=ON && cmake --build --preset default` 成功
3. 更新 README／CLAUDE.md：記錄 `--dump-costs` 與 `pdaf_cost_viz`（`-DPDAF_BUILD_GUI=ON`、Linux 需 OpenGL/X11 dev 標頭）
4. 用 superpowers:finishing-a-development-branch 收尾
