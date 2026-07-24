# M2 cost sequence 視覺化驗證工具 設計文件

日期：2026-07-24
狀態：已與使用者確認

## 目標

建立一個獨立的視覺化驗證小工具，把餵給 `ParabolicDepthEstimator`（M2）的 cost sequence 展成圖，並在圖上標示計算結果（disparity 位置）與計算過程的中間數據（cmin、mean、basin、三點、sharp、unamb 等）。為了能實際測試工具，光學模擬器要能把 cost sequence 寫成實體檔案，再餵進工具。

這是「一系列視覺化驗證工具」的第一個，先做 M2；架構要能讓日後 M1／M3 等模組的工具沿用同一套模式。

## 範疇

- 桌面 GUI 工具（Dear ImGui + ImPlot），**行程內直接跑真實的 estimator**（不重算、不做假資料），才算真正的驗證。
- 首版載入 cost-sequence 檔；載入 raw sim frame 再自己跑 M1 留作日後擴充。
- sim 端提供 `--dump-costs` 寫出實體檔案。
- 工具為 opt-in，核心 `libpdaf`／測試／CLI 維持零 GUI 相依、保持輕量。

## 資料流

```
sim ──(pdaf_cli --dump-costs <dir>)──▶ frame_XXXX.json ──▶ pdaf_cost_viz（桌面 GUI）
       M1 算 cost + 帶入真值            (replay hw_costs 格式)   載入檔 → 跑真實 estimateTraced → ImPlot 繪圖標註
```

砍掉 HTML 檢視器與中間 trace JSON 步驟：GUI 連 `libpdaf`，可在行程內直接得到真實計算結果與中間值。

## ① Estimator 透出中間值（單一真相，不重算）

在 `ParabolicDepthEstimator`（`src/algo/parabolic_depth_estimator.{h,cpp}`）新增：

```cpp
struct DepthEstimateTrace
{
  DepthEstimate result;          // 最終輸出（disparity / confidence / valid）

  // 退化旗標（成立時對應 result 為 invalid，且部分中間值未計算）
  bool degenerate_no_samples = false;  // v.size()<3 或 valid_samples<=0
  bool degenerate_flat = false;        // mean < 1e-6

  // 一般路徑的中間值
  std::size_t mi = 0;            // 最小成本索引
  float cmin = 0.f;             // 最小成本
  float mean = 0.f;             // 全曲線平均
  float depth = 0.f;            // 1 - cmin/mean（clamp）
  std::size_t basin_lo = 0, basin_hi = 0;  // basin-walk 主谷單調區間
  float second = 0.f;          // basin 外的最低成本（無競爭谷時為 +inf）
  float unamb = 0.f;           // 無歧義度
  bool boundary = false;       // 最小點在搜尋邊界

  // interior（非邊界）才有效
  float c_m1 = 0.f, c_0 = 0.f, c_p1 = 0.f;  // 三點
  float sharp = 0.f;           // 曲率
  float delta = 0.f;           // sub-pixel 頂點偏移
};

class ParabolicDepthEstimator : public IDepthEstimator
{
 public:
  DepthEstimate estimate(const CostSequence& cost) override;
  DepthEstimateTrace estimateTraced(const CostSequence& cost) const;  // 記錄中間值
};
```

`estimate()` 改成薄包裝：`return estimateTraced(cost).result;`。數學只存在 `estimateTraced()` 一處，保證工具看到的中間值與生產路徑完全一致。`IDepthEstimator` 介面不變（`estimateTraced` 是 concrete class 專有，工具直接用）。

`second` 用 `+inf` 表示「無競爭谷」；GUI 端據此不畫次低點 marker。

## ② 檔案格式（沿用 replay，向下相容）

沿用 `ReplayPdDataSource` 的 `hw_costs` frame JSON（本來就是 `CostSequence[]` + meta），另加一個**選用**的 `ground_truth_disparity` 陣列，與 `hw_costs` 平行、每條一個：

```json
{
  "meta": { "frame_id": 2, "timestamp_ms": 66.6, "lens_step_at_exposure": 300 },
  "hw_costs": [
    { "shift_min": -16, "costs": [ /* 33 個 */ ], "valid_samples": 256 }
  ],
  "ground_truth_disparity": [ -2.5 ]
}
```

`ReplayPdDataSource` 解析忽略未知欄位，故同一份檔案能同時餵 replay 與本工具。工具讀取時 `ground_truth_disparity` 缺席即視為無真值（不畫真值線）。

## ③ sim 寫出實體檔案：`pdaf_cli --dump-costs <dir>`

在 `apps/pdaf_cli/main.cpp` 的 frame loop 加入：sim 模式下，每個 frame `capture()` → M1 `compute()` → 把該 frame 的 cost sequences 以 ② 的格式寫成 `<dir>/frame_%04d.json`，`ground_truth_disparity` 由 `world.groundTruthDisparity()` 帶入（sim 才有）。

- 只在 `system.mode == "sim"` 時作用；replay 模式無 M1／無真值可寫，若指定則印警告並忽略。
- 每個 frame 寫一檔（一次跑約 5 檔），涵蓋近焦與離焦不同狀態，作為工具的真實測試資料。
- 目錄以 `std::filesystem::create_directories` 建立。

## ④ 桌面 GUI：`pdaf_cost_viz`（`tools/cost_viz/main.cpp`）

Dear ImGui + ImPlot，GLFW + OpenGL3 backend。連結 `libpdaf`。

**載入**
- 啟動可帶單一檔路徑或目錄；GUI 內用簡易檔案/目錄輸入（首版可用文字框輸入路徑 + Reload 按鈕，避免額外檔案對話框相依）。
- 載入目錄時掃 `frame_%04d.json` 連號，提供 frame 選擇器（combo / prev-next）。
- 每個 frame 可能多條 cost sequence（多 ROI），提供 ROI 選擇器。

**計算**
- 對選定的 `CostSequence` 呼叫真實 `ParabolicDepthEstimator::estimateTraced()`，取得 result + 全部中間值。

**ImPlot 主圖**（x = shift s，y = SAD cost）
- cost 離散點 + 連線
- **cmin**：最小點加強 marker
- **mean**：水平參考線（annotation 標值）
- **basin [lo,hi]**：陰影區間（ImPlot drag/shaded rect），標示 unamb 排除範圍
- **次低點**：`second` 有限時在對應點加 marker（競爭谷）
- **三點 c₋₁/c₀/c₊₁**：highlight
- **拋物線擬合**：用 c₋₁/c₀/c₊₁ 畫擬合曲線（純繪圖，係數由三點決定）
- **頂點@disparity**：垂直線 + marker 標在 `result.disparity`
- **真值**：`ground_truth_disparity` 存在時畫垂直線，並標「error = |disparity − gt|」
- 內建十字線 hover 讀精確 (s, cost)、滾輪縮放（ImPlot 原生）

**ImGui 數據面板**（表列，隨選定 sequence 更新）
- frame_id、ROI index、shift 範圍、valid_samples
- mi、cmin、mean、depth
- basin[lo,hi]、second、unamb
- boundary 旗標；interior 時 c₋₁/c₀/c₊₁、sharp、delta
- 最終 disparity、confidence、valid；退化旗標（no_samples / flat）
- 有真值時：ground truth、error

**退化情況呈現**
- `degenerate_no_samples` / `degenerate_flat`：面板明確標示 invalid 與原因，主圖仍畫 cost 曲線但不畫頂點/三點。
- `boundary`：不畫拋物線/頂點內插，標示「邊界、不內插、confidence×0.5」。

## 相依與建置（opt-in）

- 根 `CMakeLists.txt` 加 `option(PDAF_BUILD_GUI "Build ImGui/ImPlot desktop tools" OFF)`；僅 `PDAF_BUILD_GUI=ON` 時 `add_subdirectory(tools)`。
- `tools/CMakeLists.txt` 以 FetchContent 取 Dear ImGui、ImPlot、GLFW（與現有 GoogleTest 一致）；OpenGL 用 `find_package(OpenGL REQUIRED)`。ImGui/ImPlot 無 CMake，將其來源檔加入 `pdaf_cost_viz` target；GLFW 有 CMake。
- Linux 需 X11 / OpenGL dev 標頭（README/CLAUDE 註明）。
- 預設建置（`cmake --preset default`）**不含 GUI**，核心 `libpdaf`／`pdaf_tests`／`pdaf_cli` 維持零 GUI 相依；格式 CI 不受影響。
- 啟用範例：`cmake --preset default -DPDAF_BUILD_GUI=ON`。

## ⑤ 測試（headless，不需 GUI）

- **一致性**：多組 cost 輸入下 `estimateTraced(c).result == estimate(c)`（disparity、confidence、valid 皆相等）。
- **中間值自洽**：如 `depth == clamp(1 - cmin/mean)`；interior 時 `basin_lo <= mi <= basin_hi`；`c_0 == cmin`。
- **golden**：餵一組已知 cost sequence，斷言 trace 關鍵欄位（cmin、mean、mi、basin、sharp、unamb、delta、disparity）符合手算值。
- **sim dump 整合（CTest）**：`--dump-costs` 產出 `frame_%04d.json` 存在、可被解析、欄位齊全（含 ground_truth_disparity），不啟動 GUI。
- GUI 視窗本身不做自動測試（慣例）；所有計算邏輯已由上述 headless 測試涵蓋，GUI 僅繪圖。

## 目錄結構

```
src/algo/parabolic_depth_estimator.{h,cpp}   DepthEstimateTrace + estimateTraced（estimate 改薄包裝）
apps/pdaf_cli/main.cpp                        --dump-costs
tools/cost_viz/main.cpp                       ImGui+ImPlot 桌面 GUI（pdaf_cost_viz）
tools/CMakeLists.txt                          FetchContent imgui/implot/glfw，target 定義
tests/test_depth_estimator.cpp                estimateTraced 一致性 + 中間值自洽 + golden
tests/... 或 test_cost_dump.cpp               --dump-costs 整合測試
CMakeLists.txt                                option(PDAF_BUILD_GUI) + 條件 add_subdirectory(tools)
docs/tools/（未來）                            其他模組工具沿用同模式
```

## 錯誤處理原則（延續框架慣例）

- 檔案讀取／JSON 解析錯誤：GUI 於面板顯示明確錯誤訊息，不崩潰。
- estimator 執行期不丟例外（既有保證）；退化情況以旗標呈現。
- config／CLI 錯誤：`--dump-costs` 目錄無法建立時 fail-fast 並指出。

## 日後擴充（非本次範疇）

- GUI 載入 raw sim frame（含 L/R 樣本），行程內跑 M1 → M2，串起前端。
- M1／M3 各自的 trace 與 GUI（沿用同模式）。
- GUI 內直接呼叫 sim 產生互動式掃描（改物距/初始 step 即時重算）。
