# 參考來源：Qualcomm 相機 AF 架構

這個資料夾收集 PDAF_flow 架構所**對照**的 Qualcomm 相機自動對焦（AF）堆疊的公開來源。

## 誠實聲明（先讀這段）

- `docs/qualcomm_arch.svg` 那張對比圖是**依公開資料綜合整理的概念示意圖，不是任何一份 Qualcomm 官方架構圖的轉錄**。Qualcomm 內部真正的 AF 演算法架構是專有、不公開的。
- 本清單列的是**公開、可驗證**的相關來源——用來說明「市面典型方案長什麼樣」，不是宣稱這些就是設計時逐份精讀的一手文件。
- 下方每個連結都經過查證確認存在、且（專利）受讓人確為 Qualcomm（有一個常被誤列為 Qualcomm 的專利已排除，見文末）。
- 本資料夾**只放連結與對照說明，不夾帶下載的 PDF**：Qualcomm 行銷/技術文件有版權不宜重新散布；美國專利為公開紀錄，直接連到 Google Patents / USPTO 即可。

## 一、Qualcomm 官方來源

| 來源 | 說明 | 連結 |
|---|---|---|
| Qualcomm Spectra ISP（Snapdragon 820 infographic） | 官方文件，說明 Spectra ISP 的 **hybrid PDAF / laser / contrast AF** 框架——對應本專案「多模態仲裁」的概念（HAF）。 | <https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/snap820_spectraisp_infographic_fnl.pdf> |
| Qualcomm Spectra ISP 概觀 | 官方產品頁，Spectra ISP 支援 PDAF / dual-pixel sensor。 | <https://www.qualcomm.com/products/features/camera> |
| CHI API Reference（文件編號 **80-PC212**，*Qualcomm Spectra ISP Camera CHI API Reference*） | CamX-CHI 相機 HAL 的介面文件；定義 stats / AF 演算法節點如何掛入 pipeline。屬 Qualcomm 開發者文件，需透過官方管道取得，無公開直連。 | 文件編號 `80-PC212-1`（Qualcomm 文件庫） |

> 註：CamX-CHI 原始碼位於 Android BSP 的 `vendor/qcom/proprietary/camx` 與 `chi-cdk`，屬廠商 proprietary，非單一公開 repo。

## 二、Qualcomm 專利（一手來源，受讓人皆已查證為 Qualcomm Inc）

這些專利用 Qualcomm 自己的語彙描述了與本專案三模組高度對應的機制，是最扎實的公開一手來源：

| 專利 | 標題 | 對應本專案 |
|---|---|---|
| [US10044926B2](https://patents.google.com/patent/US10044926B2/en) | Optimized phase detection autofocus (PDAF) processing | **M1 / ROI**：sparse PD pixel（約 1–3% 像素）、依 confidence 優先處理中央 ROI |
| [US11314150B2](https://patents.google.com/patent/US11314150B2/en) | Phase detection autofocus (PDAF) optical system | **M2**：非對稱光圈拉大 L/R 質心分離 → disparity 與 defocus 的對應關係 |
| [US10387477B2](https://patents.google.com/patent/US10387477B2/en) | Calibration for phase detection auto focus (PDAF) camera systems | **M3 / DCC**：記錄 phase difference、lens position、confidence，由此更新對焦換算係數——與 `DccLensMapper` 的 DCC 錨點校正同一概念 |
| [US10313579B2](https://patents.google.com/patent/US10313579B2/en) | Dual phase detection auto focus camera sensor data processing | **PdInput / HAL**：dual-PD sensor 的 PD 資料傳輸與壓縮（對應 2PD 型態與資料來源抽象） |

## 三、第三方 / 社群解說（次要來源，可信度不一）

CamX-CHI 架構的公開解說多為社群整理，並非官方，僅供理解分層概念：

- CamX-CHI 架構介紹（社群整理）：<https://www.mo4tech.com/deep-understanding-of-qualcomm-camx-chi-architecture.html>
- Qualcomm HAL3 / CamX 架構學習：<https://blog.actorsfit.com/a?ID=01650-7080e718-bd5c-47df-b576-df59f2f8c206>

## 四、PDAF 一般背景（非 Qualcomm 專屬）

- Dual Pixel AF 與 PDAF 的差異（Android Authority）：<https://www.androidauthority.com/dual-pixel-autofocus-explained-1102293/>
- Qualcomm ISP 解說（Android Authority）：<https://www.androidauthority.com/qualcomm-isp-explained-999585/>

## 對照總表

| 本專案 | Qualcomm 典型 | 主要佐證來源 |
|---|---|---|
| M1 `SadCostEngine`（LRC 校正 + cost） | ISP PD 前端 stats + sparse PD | US10044926B2 |
| M2 `ParabolicDepthEstimator`（disparity + confidence） | PDLib 估測 | US11314150B2 |
| M3 `DccLensMapper`（DCC → VCM step） | PDAF 校正（PD↔lens 換算） | US10387477B2 |
| `PdInput`（raw / hw_costs） | dual-PD / ISP HW stats | US10313579B2、Spectra ISP |
| `PdafPipeline` / `AfController`（仲裁縫 + 狀態機） | HAF hybrid AF core | Spectra ISP infographic |

## 排除紀錄（透明起見）

- **US10070042B2**「Method and system of self-calibration for phase detection autofocus」在搜尋時出現，概念（PDAF 自校正）雖相關，但**受讓人為 Intel / Tahoe Research，並非 Qualcomm**，故不列入本清單，僅在此記錄以免日後被誤引。
