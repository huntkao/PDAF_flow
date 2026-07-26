#!/usr/bin/env python3
"""將 synth_repeat_pattern_experiment.py 產生的 data/*.csv（shift,cost 兩欄格式）
轉成 pdaf_cost_viz 讀取的 --dump-costs JSON 格式（frame_%04d.json），
方便直接用 build/tools/pdaf_cost_viz 開啟合成 cost 曲線、對照 count_near 標記。

用法：
  python3 csv_to_dump.py <輸出目錄> <input1.csv> [input2.csv ...]
  # 每個 input csv 對應一個 frame_%04d.json，依傳入順序編號

範例：
  python3 docs/m2-repeat-pattern-confidence/tools/csv_to_dump.py \
      /tmp/repeat_pattern_dump \
      docs/m2-repeat-pattern-confidence/data/synth_1_competitor_r1.15.csv \
      docs/m2-repeat-pattern-confidence/data/synth_2_competitors_periodic_r1.15.csv \
      docs/m2-repeat-pattern-confidence/data/synth_4_competitors_periodic_r1.15.csv
  ./build/tools/pdaf_cost_viz /tmp/repeat_pattern_dump
"""
import csv
import json
import sys
from pathlib import Path


def convert(csv_path: str, frame_id: int) -> dict:
    with open(csv_path, newline="") as f:
        rows = list(csv.reader(f))
    shifts = [int(r[0]) for r in rows[1:]]
    costs = [float(r[1]) for r in rows[1:]]
    return {
        "meta": {"frame_id": frame_id, "timestamp_ms": 0.0, "lens_step_at_exposure": 0},
        "hw_costs": [{"shift_min": min(shifts), "costs": costs, "valid_samples": len(costs)}],
        # 合成資料沒有模擬器等級的 ground truth，故不輸出 ground_truth_disparity（此欄位為選填）
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
        print(f"{csv_path} -> {outpath}")


if __name__ == "__main__":
    main()
