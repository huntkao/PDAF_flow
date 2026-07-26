# M2 Repeat Pattern 信心盲點 — 合成資料推演

延續 [`../m2-estimator-hw-cost-characteristics/`](../m2-estimator-hw-cost-characteristics/)（真實硬體 cost 序列分析），
但這裡不是量測而是**推演**：真實硬體那批資料恰好沒有出現多個競爭谷同時逼近主谷的情境，看不出問題；
這裡改用自製合成 cost 曲線，把「競爭谷數量」單獨拉出來測試，驗證現有 `ParabolicDepthEstimator::estimateTraced()`
的 `unamb`（次低點歧義指標）對 repeat pattern（週期性重複紋理造成多個近乎打平的候選谷）有沒有真正的盲點。

- `repeat_pattern_confidence_experiment.html` — 分析頁面：實驗設計、結果表、組合機率模型驗證、結論。
- `tools/synth_repeat_pattern_experiment.py` — 合成資料產生器 + 實驗工具：
  用高斯凹陷疊加建構 cost 曲線（固定主谷與競爭谷的深度比，只改變競爭谷數量），
  跑 python 移植版 `estimateTraced()` 算 `unamb`，再用 Monte Carlo 疊加雜訊統計 argmin 誤判率（AF hunting risk）。
- `data/` — 產生的合成 cost 曲線 csv（`shift,cost`），供重跑或用 `pdaf_cost_viz` 檢視個別曲線形狀。

**結論**：現有 `unamb` 只取全域唯一次低點，量得到「最深的競爭者有多深」，量不到「有幾個」。
合成實驗證明 hunting 風險隨競爭谷數量近似指數式放大，但 `unamb` 對此幾乎無感——這是待補的盲點，
建議新增獨立的 `count_near` 欄位交給下游仲裁策略使用，而不是塞進單一 `confidence` scalar。

**目前狀態**：`count_near` 已實作於 `DepthEstimateTrace`（`src/algo/parabolic_depth_estimator.{h,cpp}`），
定義為 basin 外、深度落在 `second`（現有次低點）10% 容忍帶內的局部極小值個數，純診斷欄位，
`estimate()`/`confidence` 公式與 `DepthEstimate` 公開介面都尚未改動。單元測試見
`tests/test_depth_estimator.cpp`（`DepthEstimatorTrace.CountNear*`），其中
`CountNearCountsMultipleNearTiedCompetitors` 用「unamb 完全相同、count_near 不同」的一組配平範例
直接示範現有 `unamb` 的盲點。**是否要接入 confidence 公式或 `AfController` 仲裁策略，留待後續評估。**

## 重現實驗

```
python3 docs/m2-repeat-pattern-confidence/tools/synth_repeat_pattern_experiment.py \
    docs/m2-repeat-pattern-confidence/data

# 額外做 C++ 生產路徑交叉驗證（run_estimate 建置方式見
# docs/m2-estimator-hw-cost-characteristics/README.md）：
g++ -std=c++17 -O2 -Iinclude -Isrc \
    docs/m2-estimator-hw-cost-characteristics/tools/run_estimate.cpp \
    src/algo/parabolic_depth_estimator.cpp -o /tmp/run_estimate

python3 docs/m2-repeat-pattern-confidence/tools/synth_repeat_pattern_experiment.py \
    docs/m2-repeat-pattern-confidence/data --run-estimate /tmp/run_estimate
```
