#!/usr/bin/env bash
# 用真實硬體 window_cost*.csv 檢查 count_near：印出每個 ROI 的 unamb/count_near 數值表，
# 再轉成 pdaf_cost_viz 的 dump 格式並直接開圖。
# 用法：scripts/check_count_near_hw.sh [csv1 csv2 ...]
#   不帶參數則預設用 costs/PDE/window_cost.csv costs/PDE/window_cost21.csv
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ "$#" -gt 0 ]; then
  CSVS=("$@")
else
  CSVS=(costs/PDE/window_cost.csv costs/PDE/window_cost21.csv)
fi

cmake --preset default -DPDAF_BUILD_GUI=ON
cmake --build --preset default

g++ -std=c++17 -O2 -Iinclude -Isrc \
    docs/m2-estimator-hw-cost-characteristics/tools/print_count_near.cpp \
    src/algo/parabolic_depth_estimator.cpp \
    -o build/print_count_near

echo "=== count_near / unamb ==="
build/print_count_near "${CSVS[@]}"

DUMP_DIR="costs/PDE/dump"
python3 docs/m2-estimator-hw-cost-characteristics/tools/csv_to_dump.py "$DUMP_DIR" "${CSVS[@]}"

echo "=== 開 pdaf_cost_viz ==="
./build/tools/pdaf_cost_viz "$DUMP_DIR"
