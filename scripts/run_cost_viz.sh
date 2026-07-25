#!/usr/bin/env bash
# M2 cost sequence 視覺化驗證工具：建置（GUI opt-in）→ dump costs → 開視覺化 GUI
# 用法：scripts/run_cost_viz.sh [config json] [dump 目錄]
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

CONFIG="${1:-config/default.json}"
DUMP_DIR="${2:-costs}"

cmake --preset default -DPDAF_BUILD_GUI=ON
cmake --build --preset default

./build/apps/pdaf_cli --config "$CONFIG" --out out --dump-costs "$DUMP_DIR"

./build/tools/pdaf_cost_viz "$DUMP_DIR"
