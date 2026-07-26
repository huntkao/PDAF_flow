# AfController 生命週期與 tuning 邊界測試

- `af_controller_state_machine.html` — 簡報用單頁文件：`AfController` 狀態轉移圖、四組情境（mid/near/far/retry-fail）的真實 frame-by-frame 資料、以及下方三組 tuning 參數的邊界行為測試。開啟即可瀏覽，無外部相依。附錄含 `AfController::onFrame()`（`src/control/af_controller.cpp` L47–L79）原始碼。
- `configs/` — 產生上述邊界測試數據所用的 config，皆為完整可執行的 `pdaf_cli` 設定檔（複製自 `config/default.json` 或 `config/far.json`，只改動對應的 `tuning` 欄位），依測試的參數分三個子資料夾：
  - `in_focus_disparity_max_iterations/` — 對焦容忍區與最大移動次數的邊界
  - `confidence_threshold_max_retries/` — 信心門檻與重試次數的邊界
  - `shift_window/` — M1/M2 搜尋窗（`shift_min`/`shift_max`）的邊界

## 重新產生數據

```
./build/apps/pdaf_cli --config docs/af-lifecycle-boundary/configs/<子資料夾>/<檔名>.json --out <輸出目錄>
```

`out/frames.csv` 即為 HTML 文件中各情境時間軸所使用的原始資料。
