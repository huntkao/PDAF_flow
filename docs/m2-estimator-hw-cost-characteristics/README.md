# M2 ParabolicDepthEstimator × 真實硬體 cost 序列特性

聚焦 M2 演算法本身，與 `docs/af-lifecycle-boundary/`（`AfController` 狀態機）無關。

- `estimator_characteristics.html` — 分析頁面：把 8 個使用者定義 ROI（search range 65，對稱窗 ±32）的真實硬體 cost 序列
  餵給生產路徑的 `ParabolicDepthEstimator::estimateTraced()`，記錄實測的 disparity / confidence / depth / unamb / sharp，
  整理出各序列在窗口內的週期性競爭谷（真實資料上的 aliasing 現象），並附上用 `pdaf_cost_viz` 直接視覺化這批資料的截圖與操作步驟。
- `tools/run_estimate.cpp` — 產生數據表格的小工具：讀取一批「單條 cost 序列」csv（欄位 `shift,cost`），
  對每條呼叫 `ParabolicDepthEstimator::estimateTraced()` 並印出結果。
- `tools/csv_to_dump.py` — 把硬體匯出的 `window_cost*.csv`（每列一個 ROI）轉成 `pdaf_cost_viz` 讀取的
  `--dump-costs` JSON 格式（`frame_%04d.json`），**不需改動任何專案原始碼**即可直接用既有的 GUI 工具開啟、
  互動式檢視每條 cost 曲線（basin / parabola fit / vertex / disparity 估計）。
- `assets/pdaf_cost_viz_screenshot.png` — 用 `csv_to_dump.py` 轉出的資料開 `pdaf_cost_viz` 的實際截圖，
  frame 0 / ROI 0（`user_0`）算出的 disparity/confidence 與文件表格數字一致。

## 重新產生數據

文字結果：

```
g++ -std=c++17 -O2 -Iinclude -Isrc \
    docs/m2-estimator-hw-cost-characteristics/tools/run_estimate.cpp \
    src/algo/parabolic_depth_estimator.cpp \
    -o run_estimate

./run_estimate costs/PDE/window_cost/user_*.csv costs/PDE/window_cost21/user_*.csv
```

互動視覺化（沿用專案既有的 `pdaf_cost_viz`，需先用 `-DPDAF_BUILD_GUI=ON` 建置一次）：

```
cmake --preset default -DPDAF_BUILD_GUI=ON
cmake --build --preset default

python3 docs/m2-estimator-hw-cost-characteristics/tools/csv_to_dump.py \
    costs/PDE/dump costs/PDE/window_cost.csv costs/PDE/window_cost21.csv

./build/tools/pdaf_cost_viz costs/PDE/dump
```

原始硬體 cost csv（`costs/PDE/`，含拆解後的單條序列與 `csv_to_dump.py` 產生的 dump）都在 `.gitignore` 的 `costs/` 規則下，
不隨 repo 分發；需要自行把硬體資料放到 `costs/PDE/window_cost.csv` / `window_cost21.csv`
（每列一個 ROI，欄位為 `user_N,cost_0,cost_1,...`，search range 65）。
