#!/usr/bin/env python3
"""將硬體匯出的 window_cost*.csv（每列一個 ROI：user_N,cost_0,cost_1,...）轉成
pdaf_cost_viz 讀取的 --dump-costs JSON 格式（frame_%04d.json），
不需改動任何專案原始碼即可直接用 build/tools/pdaf_cost_viz 開啟。

用法：
  python3 csv_to_dump.py <輸出目錄> <input1.csv> [input2.csv ...]
  # 每個 input csv 對應一個 frame_%04d.json，依傳入順序編號

範例：
  python3 docs/m2-estimator-hw-cost-characteristics/tools/csv_to_dump.py \
      costs/PDE/dump costs/PDE/window_cost.csv costs/PDE/window_cost21.csv
  ./build/tools/pdaf_cost_viz costs/PDE/dump
"""
import csv
import json
import sys
from pathlib import Path

SHIFT_MIN = -32  # search range 65，對稱窗 -32..32


def convert(csv_path: str, frame_id: int) -> dict:
    with open(csv_path, newline="") as f:
        rows = list(csv.reader(f))
    hw_costs = []
    for row in rows:
        values = [float(v) for v in row[1:]]
        hw_costs.append({"shift_min": SHIFT_MIN, "costs": values, "valid_samples": len(values)})
    return {
        "meta": {"frame_id": frame_id, "timestamp_ms": 0.0, "lens_step_at_exposure": 0},
        "hw_costs": hw_costs,
        # 硬體資料沒有模擬器等級的 ground truth，故不輸出 ground_truth_disparity（此欄位為選填）
    }


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    outdir = Path(sys.argv[1])
    outdir.mkdir(parents=True, exist_ok=True)
    for i, csv_path in enumerate(sys.argv[2:]):
        doc = convert(csv_path, i)
        outpath = outdir / f"frame_{i:04d}.json"
        with open(outpath, "w") as f:
            json.dump(doc, f)
        print(f"{csv_path} -> {outpath}  ({len(doc['hw_costs'])} ROI)")


if __name__ == "__main__":
    main()
