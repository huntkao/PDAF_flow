# 參考來源：Qualcomm 相機 AF 架構

這個資料夾收集 PDAF_flow 架構所**對照**的 Qualcomm 相機自動對焦（AF）堆疊的公開來源。

## 誠實聲明（先讀這段）

- `docs/qualcomm_arch.svg` 那張對比圖是**依公開資料綜合整理的概念示意圖，不是任何一份 Qualcomm 官方架構圖的轉錄**。Qualcomm 內部真正的 AF 演算法架構是專有、不公開的。
- **技術實質幾乎全部來自下方的「專利」**。經實際點閱查核，Qualcomm 官方**公開**的相機資料多屬行銷／概覽層級，**不含** PDAF 演算法架構細節；真正描述機制的一手文件（如 CHI API Reference）需透過官方管道取得、無公開下載，本文件無法查證其內容。
- 下方每個連結都經過實際點閱查核，並如實標註「實際點進去看到什麼」。
- 本資料夾**只放連結與對照說明，不夾帶下載的 PDF**：Qualcomm 文件有版權不宜重新散布；美國專利為公開紀錄，直接連到 Google Patents 即可。

## 一、Qualcomm 專利（唯一具技術實質、且可驗證的一手來源）

這些是本對照的**主要依據**。專利用 Qualcomm 自己的語彙描述了與本專案三模組高度對應的機制，受讓人皆已逐一點閱確認為 Qualcomm Inc：

| 專利 | 標題 | 對應本專案 | 查核到的關鍵內容 |
|---|---|---|---|
| [US10044926B2](https://patents.google.com/patent/US10044926B2/en) | Optimized phase detection autofocus (PDAF) processing | **M1 / ROI** | sparse PD pixel（約 1–3% 像素）、依 confidence 優先處理中央 ROI |
| [US11314150B2](https://patents.google.com/patent/US11314150B2/en) | Phase detection autofocus (PDAF) optical system | **M2** | 非對稱光圈拉大 L/R 質心分離 → disparity 與 defocus 的對應 |
| [US10387477B2](https://patents.google.com/patent/US10387477B2/en) | Calibration for phase detection auto focus (PDAF) camera systems | **M3 / DCC** | 記錄 phase difference、lens position、confidence 以更新對焦換算係數——與 `DccLensMapper` 的 DCC 錨點校正同一概念 |
| [US10313579B2](https://patents.google.com/patent/US10313579B2/en) | Dual phase detection auto focus camera sensor data processing | **PdInput / HAL** | dual-PD sensor 的 PD 資料傳輸與壓縮（對應 2PD 型態與資料來源抽象） |

> 專利文字通常比行銷資料嚴謹得多，且是公開紀錄——這是本對照最扎實的依據。

## 二、Qualcomm 官方公開資料（行銷／概覽層級，技術深度有限）

如實記錄「實際點進去看到什麼」，以免高估其內容：

| 來源 | 實際查核結果 | 連結 |
|---|---|---|
| Spectra ISP infographic（Snapdragon 820） | **行銷 infographic**；PDF 為圖片式，正文無法擷取。可佐證的只有「Spectra ISP 提供 hybrid PDAF / laser / contrast AF 框架」這個**概念層級**說法，無架構細節。 | <https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/snap820_spectraisp_infographic_fnl.pdf> |
| Snapdragon Sight 相機產品頁 | **純行銷頁**；與 AF 相關的僅一句「300% faster auto-focus（AI 臉部偵測）」，無任何 PDAF 機制或架構描述。 | <https://www.qualcomm.com/products/features/camera> |
| CHI API Reference，文件編號 **80-PC212**（*Qualcomm Spectra ISP Camera CHI API Reference*） | 被多個第三方引用、應為真實存在的 Qualcomm 開發者文件，但**無公開下載連結**，本文件**無法查證其內容**，僅記錄編號。 | 文件編號 `80-PC212-1`（Qualcomm 文件庫，需官方管道） |
| RB5 Platform 軟體參考手冊 — Camera / CHI 章節 | 官方公開技術文件頁（標題已確認存在），但內容為 JavaScript 動態載入，本次查核**未能擷取正文**，故不宣稱其 AF 技術細節。列此供有帳號/瀏覽器者自行查閱。 | <https://docs.qualcomm.com/bundle/publicresource/topics/80-88500-4/122_Camera.html> · <https://docs.qualcomm.com/bundle/publicresource/topics/80-88500-4/126_CHI.html> |

> CamX-CHI 原始碼位於 Android BSP 的 `vendor/qcom/proprietary/camx` 與 `chi-cdk`，屬廠商 proprietary，非單一公開 repo。

## 三、第三方 / 社群解說（次要，可信度不一，非官方）

CamX-CHI 分層概念的公開解說多為社群整理，僅供理解架構，不作為技術依據：

- CamX-CHI 架構介紹：<https://www.mo4tech.com/deep-understanding-of-qualcomm-camx-chi-architecture.html>
- Qualcomm HAL3 / CamX 架構學習：<https://blog.actorsfit.com/a?ID=01650-7080e718-bd5c-47df-b576-df59f2f8c206>

## 四、PDAF 一般背景（非 Qualcomm 專屬）

- Dual Pixel AF 與 PDAF 的差異（Android Authority）：<https://www.androidauthority.com/dual-pixel-autofocus-explained-1102293/>

## 對照總表（依據以專利為主）

| 本專案 | Qualcomm 典型 | 主要佐證來源（可驗證） |
|---|---|---|
| M1 `SadCostEngine`（LRC 校正 + cost） | ISP PD 前端 stats + sparse PD | US10044926B2 |
| M2 `ParabolicDepthEstimator`（disparity + confidence） | PD 估測 | US11314150B2 |
| M3 `DccLensMapper`（DCC → VCM step） | PDAF 校正（PD↔lens 換算） | US10387477B2 |
| `PdInput`（raw / hw_costs） | dual-PD / ISP HW stats | US10313579B2 |
| `PdafPipeline` / `AfController`（仲裁縫 + 狀態機） | HAF hybrid AF core | 僅 Spectra infographic 的**概念層級**說法；無公開架構文件佐證 |

## 查核與排除紀錄（透明起見）

- **US10070042B2**「Method and system of self-calibration for phase detection autofocus」在搜尋時出現、概念相關，但受讓人為 **Intel / Tahoe Research，並非 Qualcomm**，已排除。
- 上一版本曾把 Spectra infographic 與產品頁描述得像有技術細節、並替產品頁加了「PDAF / dual-pixel」字樣——經實際點閱查核，那些頁面**只有行銷內容**，已於本版更正並降級為「概覽層級」，技術依據改以專利為主。
