# PDAF_flow 設計文件

日期：2026-07-22
狀態：已與使用者確認

## 目標

建立一個 C++ 的 PDAF（相位偵測自動對焦）流程開發框架。以實際硬體產品方案的角度設計：掌控整個 AF 系統的設定、串接三個底層演算法模塊、驅動閉環對焦行為。最終產出一個有開發彈性的可 demo 框架——演算法模塊、資料來源、控制策略都可獨立替換與擴充。

## 需求範疇

- **三個演算法模塊**（目前皆無現成實作，框架定義介面並附參考實作）：
  - M1：LRC/SPC 校正套用 + 各指定 ROI 的 phase pixel matching cost sequence 計算
  - M2：由 cost sequence 推估 disparity 與 confidence
  - M3：DCC 校正資料套用，輸出 VCM step
- **資料來源**：光學模擬器閉環為主（可驗證收斂行為），raw dump 重播為輔（單模塊驗證）
- **AF 控制**：單次對焦狀態機；架構上預留 CAF（連續對焦）擴充空間
- **介面**：CLI 優先，核心與介面分離，日後可加 ImGui GUI
- **PD 型態**：抽象化，不綁 sparse PD 或 2PD；pattern 幾何由 config 描述

## 架構總覽（方案 A：分層式 AF 框架 + HAL 抽象）

四層結構，模仿實際產品 AF 架構（Qualcomm AF core / MTK AF HAL 的分層概念）：

1. **HAL 層**：`IPdDataSource`、`ILensActuator`。模擬器與 dump 重播是這兩個介面的不同實作；上層不知道資料真假，日後接真硬體只需再寫一組實作。
2. **演算法層**：M1/M2/M3 各一個純虛擬介面 + 參考實作，模塊間只透過明確定義的資料結構溝通。
3. **控制層**：`AfController` 狀態機 + `PdafPipeline`（M1+M2 組合的 estimator 包裝）+ 統一 config 系統。
4. **應用層**：CLI runner、frame loop、逐 frame CSV 記錄。

`AfController` 只依賴四個介面（兩個 HAL、estimator、M3 lens mapper），完全不知道背後是模擬器還是真硬體，也不直接碰 M1/M2——是「以實際硬體應用角度」的落實。

本設計已對照 Qualcomm 方案（CamX stats 架構、PDLib、HAF）檢視並納入四項修正：frame metadata 帶曝光當下 lens 位置、HW 統計路徑（`PdInput` variant）、動態 ROI（`AfRequest`）、HAF 式仲裁縫（`PdafPipeline`）。模塊對應關係：PDLib ≈ M1+M2+DCC 套用；本設計把 DCC 獨立為 M3，介面責任更清楚。

## 目錄結構

```
PDAF_flow/
├── include/pdaf/          # 公開介面與資料型別
│   ├── types.h            # 核心資料結構
│   ├── hal/               #   IPdDataSource, ILensActuator
│   ├── algo/              #   IPdCostEngine(M1), IDepthEstimator(M2), ILensMapper(M3)
│   └── control/           #   AfController, AfConfig
├── src/
│   ├── algo/              # 三模塊參考實作
│   ├── control/           # 狀態機、config 載入
│   ├── sim/               # 光學模擬器（實作 HAL 介面）
│   └── replay/            # dump 重播（實作 IPdDataSource）
├── apps/pdaf_cli/         # CLI runner
├── tests/                 # 單元測試（GoogleTest）
├── third_party/           # nlohmann/json (vendored)
└── config/                # 範例 JSON config
```

## 核心資料型別（`types.h`）

模塊間唯一的溝通語言：

- `PdFrame`：一個 frame 的 phase pixel 資料。抽象化設計：每個 ROI 的左/右通道 sample 陣列 + `PdPatternDesc`（pattern 幾何描述：型態、pitch、取樣佈局，由 config 提供）。**必帶 metadata：`frame_id`、`timestamp`、`lens_step_at_exposure`（曝光當下的 lens 位置）**——實機 stats 有 pipeline 延遲，M3 換算目標位置必須以量測當下的位置為基準，不能用「現在」的位置（對齊 Qualcomm stats 的 frame tagging 作法；CAF 擴充的前提）。
- `PdInput`：HAL 資料來源的輸出，tagged variant：**raw `PdFrame` 或已算好的 `CostSequence[]` 二擇一**。近代平台 PD 校正與 correlation 常由 ISP 硬體算完（M1 被硬體取代），此設計預留 HW 統計路徑；demo 全走 raw 路徑。
- `CostSequence`：M1 輸出（或由 HW 統計直接提供）。每個 ROI 一條：shift 範圍內逐點 matching cost + 有效 sample 數。
- `DepthEstimate`：M2 輸出。每個 ROI 的 disparity（sub-pixel）+ confidence（0~1）+ 判定旗標。
- `LensCommand`：M3 輸出。目標 VCM step + 預期誤差範圍。
- `LensStatus`：actuator 回報。目前 step、是否移動中。
- `AfRequest`：每個 frame 進 `onFrame()` 的請求，攜帶當前 ROI 清單（touch AF、人臉框等動態來源）；config 只提供預設 ROI。

## 模塊介面與參考實作

每個介面皆為純虛擬類別，各附一個 reference 實作：

| 模塊 | 介面 | 主要方法 | 參考實作 |
|---|---|---|---|
| M1 | `IPdCostEngine` | `init(calib, pattern)`、`compute(PdFrame, rois) → CostSequence[]` | gain 校正（LRC/SPC）後做 SAD cost |
| M2 | `IDepthEstimator` | `estimate(CostSequence) → DepthEstimate` | cost 極小值 + 拋物線內插 sub-pixel；confidence 由曲線深度/平坦度導出 |
| M3 | `ILensMapper` | `init(DccTable)`、`toLensCommand(DepthEstimate, lensStepAtExposure) → LensCommand` | disparity→defocus 線性轉換 + DCC 錨點內插；基準位置用曝光當下的 lens step |

### PdafPipeline（控制層的仲裁縫）

控制層提供 `PdafPipeline` 類別包住 M1+M2 的組合（含 `PdInput` 的 raw/HW 路徑分流），對 `AfController` 呈現為單一「focus estimator」介面（輸出 `DepthEstimate`）。controller 只認 estimator 介面，不直接依賴 M1/M2——對齊 Qualcomm HAF 把每種對焦技術（PDAF、contrast、TOF）做成插件仲裁的架構，將來加 contrast fallback 或 TOF 輔助時不需改 controller。M3 維持獨立（lens 換算不是估測技術）。

## HAL 介面

- `IPdDataSource::capture() → PdInput` — 模擬器或 replay 實作；輸出為 raw `PdFrame` 或 HW 已算好的 `CostSequence[]`（tagged variant），並一律附 frame metadata（frame_id、timestamp、lens_step_at_exposure）
- `ILensActuator::moveTo(step)` / `getStatus() → LensStatus` — 模擬器實作含 settle time 模擬；replay 模式用 null actuator

## AfController 狀態機

單次 AF，預留 CAF 擴充：

```
IDLE ──trigger()──▶ MEASURING ──▶ MOVING ──▶ SETTLING ──▶ VERIFYING ──▶ FOCUSED
                        ▲                                      │
                        └──────── 未收斂，重新量測 ◀───────────┘
                                                               └──▶ FAILED
```

- `MEASURING`：capture → `PdafPipeline`（M1 → M2，或 HW 統計直入 M2），取得 disparity/confidence
- `MOVING`：confidence 足夠 → M3 產生 `LensCommand` 下給 actuator；不足 → 重試，超過 `max_retries` 進 `FAILED`
- `SETTLING`：等 actuator 回報停止（VCM 物理 settle；實際產品必等此步）
- `VERIFYING`：再量一次，`|disparity| < in_focus_threshold` 即 `FOCUSED`；否則帶新量測回修，超過 `max_iterations` 進 `FAILED`

驅動方式為逐 frame 的 `AfController::onFrame(AfRequest)`——產品 AF 的實際型態（每個 sensor frame 踢一次狀態機），`AfRequest` 攜帶當前 frame 的動態 ROI。日後 CAF 只需在 `FOCUSED` 後加場景監測轉移，不改架構。每次 `onFrame` 輸出一筆 `AfFrameLog`（狀態、disparity、confidence、lens step 等）供記錄。

## Config 系統

單一 JSON，啟動時載入，四個區塊對應實際產品 tuning 分工：

- `sensor`：PD pattern 描述（型態、pitch）與預設 ROI 幾何（執行期 ROI 由 `AfRequest` 動態帶入，config 僅為預設值）
- `calibration`：LRC/SPC gain 表、DCC 錨點表（demo 用內建預設值，可指向外部檔案）
- `tuning`：cost shift 範圍、confidence 門檻、in-focus 門檻、retry/iteration 上限
- `system`：資料來源選擇（sim/replay）、模擬器場景參數、log 路徑

## 光學模擬器（`sim/`）

薄透鏡模型閉環：

1. 場景設定物距；由目前 VCM step 反推 lens 位置 → 計算 defocus 量
2. 對內建紋理做對應模糊 + 依 defocus 產生左右相位偏移
3. 依 `PdPatternDesc` 取樣成 `PdFrame`，加入可調 noise 與 gain 不對稱（讓 M1 的校正有作用對象)
4. actuator 端模擬 settle 需數個 frame

整個「量測→移動→收斂」閉環物理一致，可驗證收斂行為與 ground truth 誤差。

## Replay 模式（`replay/`）

讀取目錄下逐 frame 的 dump 檔（定義簡單 binary/CSV 格式）。null actuator 只記錄命令、不影響資料。用途：單模塊驗證、真機資料離線分析。

## CLI（`apps/pdaf_cli`）

```
pdaf_cli --config config/default.json [--mode sim|replay] [--out run1/]
```

- 流程：載 config → 組裝（依 mode 選 HAL 實作、注入三模塊）→ trigger AF → 逐 frame 跑到 FOCUSED/FAILED → 輸出
- 輸出：`frames.csv`（逐 frame 狀態/disparity/confidence/lens step/真實 defocus，sim 模式可比對 ground truth）、`summary.txt`（收斂與否、iteration 數、最終誤差）
- 模擬場景（物距、初始 lens 位置、noise 強度）由 `system.sim` 控制，可批次掃描場景驗證強健性

## 測試

GoogleTest：

- **單元測試**：M1 給合成 L/R 資料驗證 cost 極小值位置；M2 給已知形狀 cost curve 驗證 sub-pixel 精度與 confidence 行為；M3 給 DCC 錨點驗證內插
- **閉環整合測試**：模擬器 + 全 pipeline，斷言在數個場景（近距/遠距/低對比）於 N 個 iteration 內收斂到門檻內——框架的「活規格」

## 建置

- CMake、C++17
- 核心編成 `libpdaf`（static）；CLI 與測試連結它；日後 ImGui GUI 是另一個連 `libpdaf` 的 app，核心零改動
- 相依：nlohmann/json（header-only，vendored 進 `third_party/`）、GoogleTest（FetchContent）
- 不依賴 OpenCV；模擬器影像運算自行實作，保持輕量

## 錯誤處理原則

- Config 錯誤：啟動即 fail-fast 並指出欄位
- 執行期異常（如 ROI 無有效 sample）：不丟例外，反映為 confidence=0 走狀態機 retry 路徑——與實際產品相同，AF 執行期不崩潰、只降級
